# Servo motor firmware (MicroPython) for the PSOC(TM) Edge board.
#
# Save this file as main.py on the board. It waits for text commands from the
# ROS bridge over USB, using the same protocol as the other three boards:
#   HELLO    -> "HELLO servo 1" and READY / BUSY <id>
#   PING     -> PONG
#   RUN <id> -> STARTED <id>, the servo moves for 2 s, then DONE <id>
#   STOP     -> STOPPED <id>
# During a run, the servo gets 1250 us pulses for 1 s and 1750 us pulses for
# 1 s, so it moves to one side and then to the other.

import select
import sys
import time
from machine import Pin

MOTOR_NAME = "servo"
RUN_MS = 2000
PERIOD_US = 20000        # standard servo signal: one pulse every 20 ms
MAX_ID = 4294967295
MAX_LINE = 47


class MotorDevice:
    def __init__(self):
        self.active_id = 0        # job ID of the current run, 0 = stopped
        self.last_id = 0          # the same job is never started twice
        self.started_ms = 0
        self.last_pulse_us = 0
        self.signal = Pin("P16_1", Pin.OUT, value=0)

    def outputs_off(self):
        self.signal.value(0)

    def finish(self, event):
        """Stop the pulses and report DONE or STOPPED with the job ID."""
        self.outputs_off()
        job = self.active_id
        self.active_id = 0
        print(event, job)

    def hello(self):
        print("HELLO", MOTOR_NAME, 1)
        if self.active_id:
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
            print("STARTED", job)

    def update(self):
        """Stop after 2 s, otherwise send one servo pulse every 20 ms."""
        if not self.active_id:
            return
        elapsed = time.ticks_diff(time.ticks_ms(), self.started_ms)
        if elapsed >= RUN_MS:
            self.finish("DONE")
            return
        now = time.ticks_us()
        if time.ticks_diff(now, self.last_pulse_us) < PERIOD_US:
            return
        self.last_pulse_us = now
        pulse_us = 1250 if elapsed < 1000 else 1750
        try:
            self.signal.value(1)
            time.sleep_us(pulse_us)
        finally:
            self.signal.value(0)


def run():
    device = MotorDevice()
    line = ""
    discard = False
    try:
        # poll() lets us check for serial input without blocking the pulses.
        poller = select.poll()
        poller.register(sys.stdin, select.POLLIN)
        device.hello()
        while True:
            device.update()
            for _ in range(32):
                if not poller.poll(0):
                    break
                char = sys.stdin.read(1)
                if not char:
                    raise OSError("serial input closed")
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
                device.update()
            time.sleep_ms(1)
    finally:
        device.outputs_off()


if __name__ == "__main__":
    run()
