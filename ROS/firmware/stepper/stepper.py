# Stepper motor firmware (MicroPython) for the PSOC(TM) 6 board.
#
# Save this file as main.py on the board. It waits for text commands from the
# ROS bridge over USB, using the same protocol as the other three boards:
#   HELLO    -> "HELLO stepper 1" and READY / BUSY <id> / FAULT <reason>
#   PING     -> PONG
#   RUN <id> -> STARTED <id>, the motor steps for 2 s, then DONE <id>
#   STOP     -> STOPPED <id>
# The board stops the motor by itself after 2 s, even if the PC is gone.

import sys
import time
from machine import Pin, Timer

MOTOR_NAME = "stepper"
RUN_MS = 2000
STEP_PERIOD_US = 10000   # one step every 10 ms
PULSE_US = 50            # length of one STEP pulse
MAX_ID = 4294967295
MAX_LINE = 47


class MotorDevice:
    def __init__(self):
        self.active_id = 0        # job ID of the current run, 0 = stopped
        self.last_id = 0          # the same job is never started twice
        self.started_ms = 0
        self.last_pulse_us = 0
        self.failed = False
        # Stepper driver pins. EN is active low: 1 = driver off.
        self.enable = Pin("P9_2", Pin.OUT, value=1)
        self.step = Pin("P9_0", Pin.OUT, value=0)
        self.direction = Pin("P9_1", Pin.OUT, value=0)

    def outputs_off(self):
        self.enable.value(1)
        self.step.value(0)

    def finish(self, event):
        """Stop the motor and report DONE or STOPPED with the job ID."""
        self.outputs_off()
        job = self.active_id
        self.active_id = 0
        print(event, job)

    def hello(self):
        print("HELLO", MOTOR_NAME, 1)
        if self.failed:
            print("FAULT TIMER_UPDATE")
        elif self.active_id:
            print("BUSY", self.active_id)
        else:
            print("READY")

    def command(self, line):
        if line == "HELLO":
            self.hello()
        elif line == "PING":
            print("PONG")
        elif line == "STOP":
            self.finish("STOPPED")
        elif line.startswith("RUN "):
            self.run(line[4:])
        else:
            print("ERROR 0 UNKNOWN_COMMAND")

    def run(self, number):
        if not number or any(c < "0" or c > "9" for c in number):
            print("ERROR 0 INVALID_ID")
            return
        job = int(number)
        if not 1 <= job <= MAX_ID:
            print("ERROR 0 INVALID_ID")
        elif self.failed:
            print("ERROR", job, "TIMER_UPDATE")
        elif self.active_id:
            # A repeated RUN for the running job is answered again; others are refused.
            if job == self.active_id:
                print("STARTED", job)
            else:
                print("ERROR", job, "BUSY")
        elif job == self.last_id:
            print("ERROR", job, "DUPLICATE")
        else:
            self.last_id = job
            self.started_ms = time.ticks_ms()
            self.last_pulse_us = time.ticks_us()
            self.active_id = job
            self.enable.value(0)
            print("STARTED", job)

    def update(self):
        """Called every 1 ms by the timer: stop after 2 s, otherwise step."""
        if not self.active_id:
            return
        elapsed = time.ticks_diff(time.ticks_ms(), self.started_ms)
        if elapsed >= RUN_MS:
            self.finish("DONE")
            return
        now = time.ticks_us()
        if time.ticks_diff(now, self.last_pulse_us) < STEP_PERIOD_US:
            return
        self.last_pulse_us = now
        try:
            self.step.value(1)
            time.sleep_us(PULSE_US)
        finally:
            self.step.value(0)

    def tick(self, _timer):
        # If update() fails, switch the motor off and refuse further runs.
        completed = False
        try:
            self.update()
            completed = True
        finally:
            if not completed:
                job = self.active_id
                self.active_id = 0
                self.failed = True
                self.outputs_off()
                print("ERROR", job, "TIMER_UPDATE")


def run():
    device = MotorDevice()
    timer = None
    line = ""
    discard = False
    try:
        # The PSOC 6 port cannot poll sys.stdin, so the motor is driven by a
        # 1 ms timer while the loop below waits for the next character.
        timer = Timer(0, period=1, mode=Timer.PERIODIC, callback=device.tick)
        device.hello()
        while True:
            char = sys.stdin.read(1)
            if not char:
                raise OSError("serial input closed")
            if char == "\x03":
                raise KeyboardInterrupt
            if char == "\r":
                continue
            if char == "\n":
                if not discard and line:
                    device.command(line)
                line = ""
                discard = False
            elif not discard:
                # Invalid or too long lines stop the motor.
                if not 32 <= ord(char) <= 126 or len(line) >= MAX_LINE:
                    if device.active_id:
                        device.finish("STOPPED")
                    print("ERROR 0 INVALID_LINE")
                    line = ""
                    discard = True
                else:
                    line += char
    finally:
        # Make sure no step is sent after the script ends.
        device.active_id = 0
        try:
            device.outputs_off()
        finally:
            if timer is not None:
                timer.deinit()


if __name__ == "__main__":
    run()
