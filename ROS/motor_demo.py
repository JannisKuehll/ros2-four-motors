#!/usr/bin/env python3
"""ROS 2 demo: one central controller coordinates four motors.

Each motor (DC, BLDC, stepper and servo) has its own microcontroller that is
connected over USB. For every board, this script starts a small ROS 2 "bridge"
node that translates ROS messages into short text commands on the serial port.
A central "controller" node reads the keyboard and decides which motor runs.

  keyboard -> motor_controller --/motors/<role>/command--> <role>_motor -> USB -> board
              motor_controller <--/motors/<role>/status--- <role>_motor <- USB <- board

Serial protocol (115200 baud, one text line per message):
  HELLO     -> HELLO <role> 1, then READY, BUSY <id> or FAULT <reason>
  PING      -> PONG (heartbeat)
  RUN <id>  -> STARTED <id>, and DONE <id> after 2 seconds
  STOP      -> STOPPED <id>
  Problems  -> ERROR <id> <reason>
  BLDC only -> REARMING / ARMED while its shield is reset after a run
"""

import argparse
import os
import random
import select
import signal
import sys
import time
from dataclasses import dataclass, field

if sys.platform == "win32":
    import msvcrt
else:
    import termios
    import tty

import rclpy
import serial
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from serial.tools import list_ports
from std_msgs.msg import String

# ---------------------------------------------------------------------------
# Settings
# ---------------------------------------------------------------------------

MOTORS = ("dc", "bldc", "stepper", "servo")
KEYS = {"d": "dc", "b": "bldc", "t": "stepper", "s": "servo"}
VALID_KEYS = "dbtsrm q"

# USB vendor/product IDs of the on-board debuggers: SEGGER J-Link (XMC kits)
# and KitProg3 (PSOC kits). Only these ports are probed automatically.
USB_IDS = {(0x1366, 0x1024), (0x04B4, 0xF155)}

BAUD = 115200
MAX_LINE = 96                    # longest serial line we accept
RUN_SECONDS = 2.0                # the boards stop each motor by themselves after 2 s
REARM_TIMEOUT_SECONDS = 4.0      # BLDC shield reset after a run (~2.5 s)
STOP_TIMEOUT_SECONDS = 1.0       # a board must confirm STOP within this time
STOP_ALL_TIMEOUT_SECONDS = 4.5   # controller-side limit for "all motors stopped"
HEARTBEAT_TIMEOUT_SECONDS = 3.0  # no PONG for this long = connection lost

# Status messages are "latched": a controller that starts late still receives
# the last status of every motor.
STATUS_QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


def command_topic(role: str) -> str:
    return f"/motors/{role}/command"


def status_topic(role: str) -> str:
    return f"/motors/{role}/status"


def parse_job_id(text: str, allow_zero: bool = False) -> int:
    """Every RUN gets a job ID (uint32). ID 0 is only used by 'STOPPED 0'."""
    if not text.isascii() or not text.isdecimal():
        raise ValueError(f"Invalid job ID: {text!r}")
    number = int(text)
    if not (0 if allow_zero else 1) <= number <= 0xFFFFFFFF:
        raise ValueError(f"Job ID outside uint32 range: {text!r}")
    return number


# ---------------------------------------------------------------------------
# Serial helpers
# ---------------------------------------------------------------------------

class LineReader:
    """Collects bytes from a serial port and returns complete text lines."""

    def __init__(self) -> None:
        self.buffer = bytearray()

    def read(self, connection: serial.Serial) -> list[str]:
        self.buffer.extend(connection.read(min(connection.in_waiting, 1024)))
        lines = []
        while b"\n" in self.buffer:
            raw, _, self.buffer = self.buffer.partition(b"\n")
            if len(raw) > MAX_LINE:
                raise ValueError("Serial reply is too long")
            line = raw.rstrip(b"\r").decode("ascii")
            if line.strip():
                lines.append(line)
        if len(self.buffer) > MAX_LINE:
            raise ValueError("Serial reply without line end is too long")
        return lines


@dataclass
class Device:
    """An identified board: its role, open serial port and read buffer."""
    role: str
    connection: serial.Serial
    reader: LineReader = field(default_factory=LineReader)
    rearming_since: float | None = None  # set if the BLDC is still rearming


def write_line(connection: serial.Serial, line: str) -> None:
    data = (line + "\n").encode("ascii")
    if connection.write(data) != len(data):
        raise serial.SerialException("Serial command was not sent completely")


# ---------------------------------------------------------------------------
# Finding the four boards
# ---------------------------------------------------------------------------

def identify(port: str, expected_role: str | None = None, timeout: float = 6.0) -> Device:
    """Ask a board who it is (HELLO) and make sure its motor is stopped (STOP).

    The port is returned open on success, so no other program can grab it.
    """
    connection = serial.Serial(
        port=None, baudrate=BAUD, timeout=0, write_timeout=0.2, exclusive=True
    )
    # Keep DTR/RTS low so opening the port does not reset the board.
    connection.dtr = False
    connection.rts = False
    connection.port = port
    success = False
    try:
        connection.open()
        connection.reset_input_buffer()
        reader = LineReader()
        role = None               # filled in by the HELLO answer
        state_seen = False        # READY/BUSY/... received after HELLO
        stop_sent = False
        rearming_since = None
        next_hello = 0.0
        deadline = time.monotonic() + timeout

        while time.monotonic() < deadline:
            # Repeat HELLO until the board answers. STOP or RUN are never sent
            # to a board that has not identified itself.
            if not stop_sent and time.monotonic() >= next_hello:
                write_line(connection, "HELLO")
                next_hello = time.monotonic() + 0.5

            for line in reader.read(connection):
                parts = line.split()
                if parts[0] == "HELLO" and len(parts) == 3:
                    if parts[1] not in MOTORS or parts[2] != "1":
                        raise ValueError(f"{port}: unsupported device {line!r}")
                    if expected_role and parts[1] != expected_role:
                        raise ValueError(f"{port}: expected {expected_role}, board reports {parts[1]}")
                    role = parts[1]
                elif role is None:
                    print(f"{port}: waiting for ROS firmware; received {line!r}", flush=True)
                elif parts[0] in ("FAULT", "ERROR"):
                    raise ValueError(f"{port}: firmware reports {line}")
                elif line == "READY" or (parts[0] == "BUSY" and len(parts) == 2):
                    state_seen = True
                elif role == "bldc" and line == "REARMING":
                    state_seen = True
                    rearming_since = rearming_since or time.monotonic()
                elif role == "bldc" and line == "ARMED":
                    state_seen = True
                    rearming_since = None
                elif stop_sent and parts[0] == "STOPPED" and len(parts) == 2:
                    parse_job_id(parts[1], allow_zero=True)
                    success = True
                    return Device(role, connection, reader, rearming_since)
                else:
                    print(f"{port}: ignoring {line!r}", flush=True)

            if role is not None and state_seen and not stop_sent:
                write_line(connection, "STOP")
                stop_sent = True
            time.sleep(0.01)

        raise TimeoutError(f"{port}: no complete HELLO/STOP answer. Is the ROS firmware flashed?")
    finally:
        if not success:
            connection.close()


def discover(overrides: dict[str, str]) -> dict[str, Device]:
    """Identify all four boards, either automatically or from --port options."""
    if overrides:
        candidates = [(port, role) for role, port in overrides.items()]
    else:
        candidates = [
            (port.device, None)
            for port in list_ports.comports()
            if (port.vid, port.pid) in USB_IDS
        ]

    devices: dict[str, Device] = {}
    try:
        for port, expected_role in candidates:
            try:
                device = identify(port, expected_role)
            except (serial.SerialException, OSError, ValueError, UnicodeError) as error:
                print(f"Cannot use {port}: {error}", file=sys.stderr, flush=True)
                continue
            if device.role in devices:
                device.connection.close()
                raise ValueError(f"Two boards report the role {device.role}.")
            devices[device.role] = device
            print(f"Identified {device.role}: {port}", flush=True)

        missing = set(MOTORS) - devices.keys()
        if missing:
            raise ValueError(
                f"Missing boards: {', '.join(sorted(missing))}. Check USB cables, firmware, "
                "serial permissions and close other serial monitors."
            )
        return devices
    except BaseException:
        for device in devices.values():
            device.connection.close()
        raise


# ---------------------------------------------------------------------------
# Bridge node: ROS <-> one serial board
# ---------------------------------------------------------------------------

class MotorBridge(Node):
    """Forwards RUN/STOP from ROS to one board and publishes its answers.

    It also watches the board: every command must be confirmed in time and a
    heartbeat (PING/PONG) checks that the USB connection is still alive.
    """

    def __init__(self, device: Device) -> None:
        super().__init__(f"{device.role}_motor")
        self.device = device
        self.reader = device.reader
        self.rearming_since = device.rearming_since
        self.failed = False
        self.active_id = 0            # job ID of the current RUN, 0 = none
        self.acknowledged = False     # STARTED received for active_id
        self.run_sent_at = 0.0
        self.stop_requested_at: float | None = None
        self.stop_error_reported = False
        self.last_pong = time.monotonic()
        self.next_ping = self.last_pong

        self.publisher = self.create_publisher(String, status_topic(device.role), STATUS_QOS)
        self.subscription = self.create_subscription(
            String, command_topic(device.role), self.on_command, 10
        )
        self.timer = self.create_timer(0.02, self.poll)
        self.emit("REARMING" if self.rearming_since is not None else "READY")

    def emit(self, text: str) -> None:
        self.publisher.publish(String(data=text))

    def request_stop(self) -> None:
        if self.stop_requested_at is None:
            write_line(self.device.connection, "STOP")
            self.stop_requested_at = time.monotonic()
            self.stop_error_reported = False

    def fail(self, reason: str) -> None:
        """Stop the motor and report an error. The bridge accepts no new RUN."""
        if self.failed:
            return
        self.failed = True
        self.get_logger().error(reason)
        try:
            self.request_stop()
        except (serial.SerialException, OSError) as error:
            self.get_logger().error(f"Cannot send STOP: {error}")
        self.emit(f"ERROR {self.active_id} {reason}")

    # --- ROS -> board -------------------------------------------------------

    def on_command(self, message: String) -> None:
        line = message.data
        parts = line.split()
        try:
            if line == "STOP":
                self.request_stop()
            elif len(parts) == 2 and parts[0] == "RUN":
                number = parse_job_id(parts[1])
                if self.failed:
                    self.get_logger().error("RUN refused: this motor has a fault.")
                elif self.active_id or self.stop_requested_at is not None:
                    self.emit(f"ERROR {number} BUSY")
                elif self.rearming_since is not None:
                    self.emit(f"REJECTED {number} REARMING")
                else:
                    self.active_id = number
                    self.acknowledged = False
                    self.run_sent_at = time.monotonic()
                    write_line(self.device.connection, f"RUN {number}")
            else:
                raise ValueError(f"Invalid ROS command: {line!r}")
        except (serial.SerialException, OSError, ValueError) as error:
            self.fail(str(error))

    # --- board -> ROS -------------------------------------------------------

    def receive(self, line: str) -> None:
        parts = line.split()
        event = parts[0]

        if line == "PONG":
            self.last_pong = time.monotonic()
            return

        if self.failed and event not in ("STOPPED", "ERROR", "FAULT"):
            self.get_logger().warning(f"Ignoring {line!r} after fault; waiting for STOPPED.")
            return

        if self.device.role == "bldc" and not self.failed and self.handle_bldc_rearm(line, parts):
            return

        if event in ("ERROR", "FAULT"):
            self.fail(f"Firmware reported {line}")
        elif event == "HELLO" or line == "READY":
            # The board only sends these on its own after a reset.
            self.fail("Unexpected board restart. Check the hardware and restart this script.")
        elif event == "STARTED" and len(parts) == 2:
            if parse_job_id(parts[1]) != self.active_id:
                raise ValueError(f"STARTED for unknown job {parts[1]}")
            self.acknowledged = True
            self.emit(line)
        elif event == "DONE" and len(parts) == 2:
            if parse_job_id(parts[1]) != self.active_id or not self.acknowledged:
                raise ValueError(f"DONE for unknown job {parts[1]}")
            self.active_id = 0
            self.emit(line)
        elif event == "STOPPED" and len(parts) == 2:
            number = parse_job_id(parts[1], allow_zero=True)
            if self.stop_requested_at is None or number not in (0, self.active_id):
                raise ValueError(f"Unexpected STOPPED for job {number}")
            self.active_id = 0
            self.stop_requested_at = None
            self.emit(line)
        else:
            raise ValueError(f"Unexpected serial reply: {line!r}")

    def handle_bldc_rearm(self, line: str, parts: list[str]) -> bool:
        """BLDC only: after each run, the shield is reset for about 2.5 s.

        REARMING = reset started, ARMED = ready again. During this time, the
        board answers RUN with 'ERROR <id> REARMING' without moving the motor.
        Returns True if the line was handled here.
        """
        if line == "REARMING":
            if self.rearming_since is None:
                if not (self.active_id and self.acknowledged):
                    raise ValueError("REARMING without an active BLDC job")
                self.rearming_since = time.monotonic()
                self.emit("REARMING")
            return True
        if line == "ARMED":
            if self.rearming_since is None or self.active_id:
                raise ValueError("Unexpected ARMED")
            self.rearming_since = None
            self.emit("READY")
            return True
        if (
            len(parts) == 3 and parts[0] == "ERROR" and parts[2] == "REARMING"
            and self.active_id and not self.acknowledged
            and parts[1] == str(self.active_id)
        ):
            self.rearming_since = self.rearming_since or time.monotonic()
            self.active_id = 0
            self.emit(f"REJECTED {parts[1]} REARMING")
            return True
        return False

    # --- periodic checks (every 20 ms) ---------------------------------------

    def poll(self) -> None:
        try:
            for line in self.reader.read(self.device.connection):
                self.receive(line)
            now = time.monotonic()

            if (
                self.stop_requested_at is not None
                and now - self.stop_requested_at > STOP_TIMEOUT_SECONDS
                and not self.stop_error_reported
            ):
                self.stop_error_reported = True
                reason = "STOP not confirmed; check motor and power supply."
                if self.failed:
                    self.get_logger().error(reason)
                else:
                    self.fail(reason)
            if self.failed:
                return

            if self.active_id and not self.acknowledged and now - self.run_sent_at > 1.0:
                self.fail("START not confirmed")
            elif self.active_id and now - self.run_sent_at > RUN_SECONDS + 1.5:
                self.fail("DONE not received; motor stop is not confirmed.")
            elif self.rearming_since is not None and now - self.rearming_since > REARM_TIMEOUT_SECONDS:
                self.fail("BLDC shield rearm timeout")
            elif now - self.last_pong > HEARTBEAT_TIMEOUT_SECONDS:
                self.fail("Serial heartbeat lost")
            elif now >= self.next_ping:
                write_line(self.device.connection, "PING")
                self.next_ping = now + 1.0
        except (serial.SerialException, OSError, ValueError, UnicodeError) as error:
            if not self.failed:
                self.fail(str(error))
            elif not self.stop_error_reported:
                self.stop_error_reported = True
                self.get_logger().error(f"Cannot read STOP confirmation: {error}")


# ---------------------------------------------------------------------------
# Controller node: decides which motor runs
# ---------------------------------------------------------------------------

class Controller(Node):
    """Central coordinator. Only one motor runs at a time.

    States per motor: unknown, idle, requested, running, rearming, fault.
    """

    def __init__(self) -> None:
        super().__init__("motor_controller")
        self.command_publishers = {
            role: self.create_publisher(String, command_topic(role), 10) for role in MOTORS
        }
        self.status_subscriptions = [
            self.create_subscription(
                String, status_topic(role),
                lambda message, motor=role: self.on_status(motor, message),
                STATUS_QOS,
            )
            for role in MOTORS
        ]
        # Key presses are ROS messages too, so another node could send them.
        self.key_publisher = self.create_publisher(String, "/motor_demo/key", 10)
        self.key_subscription = self.create_subscription(
            String, "/motor_demo/key", self.on_key, 10
        )
        self.states = {role: "unknown" for role in MOTORS}
        self.active: tuple[str, int] | None = None   # (role, job ID) of the running job
        self.pending_stops: set[str] = set()
        self.random_mode = False
        self.exit_requested = False
        self.failed = False
        self.next_random_at = 0.0
        self.job_deadline = 0.0
        self.stop_deadline = 0.0
        # Random start value, so a restarted script never repeats an old job ID.
        self.sequence = random.SystemRandom().randint(1, 2**31 - 1)
        self.timer = self.create_timer(0.05, self.tick)

    def send(self, role: str, command: str) -> None:
        self.command_publishers[role].publish(String(data=command))

    def stop_all(self) -> None:
        self.random_mode = False
        self.pending_stops = set(MOTORS)
        self.stop_deadline = time.monotonic() + STOP_ALL_TIMEOUT_SECONDS
        for role in MOTORS:
            self.send(role, "STOP")

    def fail(self, reason: str) -> None:
        if not self.failed:
            self.failed = True
            self.stop_all()
            self.get_logger().error(reason)

    def job_finished(self) -> None:
        self.active = None
        self.next_random_at = time.monotonic() + 0.25

    def on_status(self, role: str, message: String) -> None:
        parts = message.data.split()
        event = parts[0] if parts else ""
        try:
            number = parse_job_id(parts[1], allow_zero=True) if len(parts) >= 2 else 0
        except ValueError as error:
            self.fail(f"{role}: {error}")
            return

        if event == "READY":
            self.states[role] = "idle"
        elif message.data == "REARMING":
            self.states[role] = "rearming"
        elif event == "REJECTED" and len(parts) == 3:
            self.states[role] = "rearming"
            if self.active == (role, number):
                self.job_finished()
            self.get_logger().warning(f"{role}: still rearming; RUN {number} skipped.")
            return
        elif event in ("STARTED", "DONE", "STOPPED") and len(parts) == 2:
            if event == "STARTED":
                self.states[role] = "running"
            elif self.states[role] != "rearming":
                self.states[role] = "idle"
            if event == "STOPPED":
                self.pending_stops.discard(role)
            if (
                event != "STARTED" and self.active is not None and self.active[0] == role
                and (self.active[1] == number or (event == "STOPPED" and number == 0))
            ):
                self.job_finished()
        elif event == "ERROR":
            self.states[role] = "fault"
            self.fail(f"{role}: {message.data}. Check the hardware.")
            return
        else:
            self.fail(f"{role}: invalid status {message.data!r}")
            return
        self.get_logger().info(f"{role}: {message.data}")

    def run_motor(self, role: str) -> None:
        if self.failed or self.pending_stops or self.active is not None:
            self.get_logger().warning("RUN ignored: another motor is running or stopping.")
            return
        if self.states[role] == "rearming":
            self.get_logger().warning(f"{role}: shield is rearming; try again in a moment.")
            return
        if any(state not in ("idle", "rearming") for state in self.states.values()):
            self.get_logger().warning("RUN ignored: waiting for all four boards to be ready.")
            return
        self.sequence = self.sequence % 0xFFFFFFFF + 1
        self.active = (role, self.sequence)
        self.job_deadline = time.monotonic() + RUN_SECONDS + 2.0
        self.states[role] = "requested"
        self.get_logger().info(f"{role}: RUN {self.sequence}")
        self.send(role, f"RUN {self.sequence}")

    def on_key(self, message: String) -> None:
        key = message.data
        if key in KEYS:
            if self.random_mode:
                self.get_logger().warning("Random mode is active. Press m to switch to manual.")
            else:
                self.run_motor(KEYS[key])
        elif key == "r":
            if self.failed or self.pending_stops or self.active is not None:
                self.get_logger().warning("Random mode not possible right now.")
            else:
                self.random_mode = True
                self.get_logger().info("Random mode on. Press m or SPACE to stop.")
        elif key in ("m", " "):
            self.stop_all()
            self.get_logger().info("STOP all motors; manual mode.")
        elif key == "q":
            self.exit_requested = True

    def tick(self) -> None:
        now = time.monotonic()
        if self.pending_stops and now > self.stop_deadline:
            self.fail(f"STOP not confirmed for {sorted(self.pending_stops)}")
        elif self.active is not None and now > self.job_deadline:
            self.fail("Job did not finish in time.")
        elif (
            self.random_mode and not self.failed and not self.pending_stops
            and self.active is None and now >= self.next_random_at
            and all(state in ("idle", "rearming") for state in self.states.values())
        ):
            idle = [role for role in MOTORS if self.states[role] == "idle"]
            if idle:
                self.run_motor(random.choice(idle))


# ---------------------------------------------------------------------------
# Keyboard input without pressing Enter (Linux and Windows)
# ---------------------------------------------------------------------------

class Keyboard:
    def __init__(self, enabled: bool) -> None:
        self.enabled = enabled
        self.saved_settings = None
        self.in_escape_sequence = False   # Linux: arrow keys etc.
        self.extended_key = False         # Windows: arrow keys etc.

    def __enter__(self):
        if self.enabled and sys.platform != "win32":
            # Linux: deliver every key immediately instead of line by line.
            self.saved_settings = termios.tcgetattr(sys.stdin.fileno())
            tty.setcbreak(sys.stdin.fileno())
        return self

    def __exit__(self, *_):
        if self.saved_settings is not None:
            termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, self.saved_settings)

    def keys(self) -> list[str]:
        """Return all keys pressed since the last call (never blocks)."""
        if not self.enabled:
            return []
        keys = []
        if sys.platform == "win32":
            while msvcrt.kbhit():
                char = msvcrt.getwch()
                if self.extended_key:
                    self.extended_key = False
                elif char in ("\x00", "\xe0"):
                    self.extended_key = True
                elif char == "\x03":
                    raise KeyboardInterrupt
                elif char in VALID_KEYS:
                    keys.append(char)
            return keys

        if not select.select([sys.stdin.fileno()], [], [], 0)[0]:
            return []
        data = os.read(sys.stdin.fileno(), 64)
        if not data:
            return ["q"]  # terminal closed
        for value in data:
            if value == 27:
                self.in_escape_sequence = True
            elif self.in_escape_sequence:
                if 64 <= value <= 126 and value not in (ord("["), ord("O")):
                    self.in_escape_sequence = False
            elif chr(value) in VALID_KEYS:
                keys.append(chr(value))
        return keys


# ---------------------------------------------------------------------------
# Main program
# ---------------------------------------------------------------------------

HELP_TEXT = """
d = DC   b = BLDC   t = stepper   s = servo   (each runs for 2 s)
r = random mode   m = manual mode   SPACE = stop all   q = quit
Only one motor runs at a time. The BLDC needs ~2.5 s after each run.
STOP is a software stop, not an emergency stop.
"""


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="ROS 2 demo for four motors.")
    parser.add_argument("--list-ports", action="store_true",
                        help="list serial ports and exit")
    parser.add_argument("--check-connections", action="store_true",
                        help="identify all four boards, confirm STOP and exit (never sends RUN)")
    parser.add_argument("--no-keyboard", action="store_true",
                        help="no terminal input; send keys to the /motor_demo/key topic instead")
    parser.add_argument("--port", action="append", default=[], metavar="ROLE=PORT",
                        help="set the port per role instead of automatic search, "
                             "e.g. --port dc=/dev/ttyACM0 (give all four)")
    args = parser.parse_args()

    args.overrides = {}
    for argument in args.port:
        role, _, port = argument.partition("=")
        if role not in MOTORS or not port or role in args.overrides:
            parser.error("--port must look like dc=PORT, bldc=PORT, stepper=PORT or servo=PORT")
        args.overrides[role] = port
    if args.overrides and (
        len(args.overrides) != len(MOTORS) or len(set(args.overrides.values())) != len(MOTORS)
    ):
        parser.error("Give four different ports, one per motor, or omit --port.")
    interactive = not (args.no_keyboard or args.check_connections or args.list_ports)
    if interactive and not sys.stdin.isatty():
        parser.error("Start this script in a terminal, or use --no-keyboard.")
    return args


def list_serial_ports() -> None:
    ports = list(list_ports.comports())
    for port in ports:
        print(port.device, port.description, f"serial={port.serial_number}")
    if not any((port.vid, port.pid) in USB_IDS for port in ports):
        print("No J-Link/KitProg3 ports found. Check the USB cables.")


def main() -> int:
    args = parse_arguments()
    if args.list_ports:
        list_serial_ports()
        return 0

    devices = discover(args.overrides)
    if args.check_connections:
        for device in devices.values():
            device.connection.close()
        print("All four boards identified and stopped. No RUN command sent.", flush=True)
        return 0

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    executor = SingleThreadedExecutor()
    controller = Controller()
    bridges = [MotorBridge(device) for device in devices.values()]
    for node in [controller, *bridges]:
        executor.add_node(node)
    print(HELP_TEXT, flush=True)

    try:
        with Keyboard(enabled=not args.no_keyboard) as keyboard:
            while (
                rclpy.ok() and not controller.exit_requested and not controller.failed
                and not any(bridge.failed for bridge in bridges)
            ):
                for key in keyboard.keys():
                    controller.key_publisher.publish(String(data=key))
                executor.spin_once(timeout_sec=0.02)
    except KeyboardInterrupt:
        print("\nStopping all motors...", flush=True)
    finally:
        # Always stop every motor before the script ends and wait for the
        # confirmations. If one is missing, the user must cut motor power.
        if rclpy.ok():
            controller.stop_all()
            while controller.pending_stops and time.monotonic() < controller.stop_deadline:
                executor.spin_once(timeout_sec=0.02)
        stop_confirmed = not controller.pending_stops
        if not stop_confirmed:
            print(
                f"STOP not confirmed for {sorted(controller.pending_stops)}. "
                "Switch off motor power; do not assume the motors have stopped.",
                file=sys.stderr, flush=True,
            )
        failed = controller.failed or any(bridge.failed for bridge in bridges)
        for device in devices.values():
            device.connection.close()
        executor.shutdown()
        for node in [controller, *bridges]:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if stop_confirmed and not failed else 1


def stop_on_signal(_signum, _frame):
    raise KeyboardInterrupt


if __name__ == "__main__":
    signal.signal(signal.SIGTERM, stop_on_signal)
    if sys.platform == "win32":
        signal.signal(signal.SIGBREAK, stop_on_signal)
    try:
        sys.exit(main())
    except (ValueError, OSError, serial.SerialException) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        sys.exit(1)
    except KeyboardInterrupt:
        print("Cancelled during startup; no RUN command sent.", file=sys.stderr)
        sys.exit(130)
