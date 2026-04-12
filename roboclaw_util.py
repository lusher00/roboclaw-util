#!/usr/bin/env python3
"""
roboclaw_util.py — RoboClaw packet serial utility
BBB: /dev/ttyO1 @ 460800, address 0x80

Usage:
  python3 roboclaw_util.py <command> [args] [options]

Commands:
  probe                      raw comms test, dumps bytes
  status                     firmware, voltages, temp, error flags
  encoders                   encoder counts and speeds
  currents                   motor currents
  pid1                       read M1 velocity PID
  pid2                       read M2 velocity PID
  setpid1 Kp Ki Kd qpps     set M1 velocity PID and save
  setpid2 Kp Ki Kd qpps     set M2 velocity PID and save
  stop                       zero both motors
  read_config                dump and decode config register
  set_config HEX             write raw config value e.g. 0x00e3
  set_timeout MS             set serial timeout in ms (0=disabled, recommend 500)
  read_timeout               read current serial timeout
  estop_config               configure S3/S4/S5 as e-stop inputs and save
  read_pins                  read S3/S4/S5/CTRL1/CTRL2 pin modes
  set_pins S3 S4 S5 C1 C2    set pin modes (hex values, e.g. 0x00 0x41 0x00 0x00 0x00)
  estop_pin [PIN] [--latch]  configure pin as e-stop (default S4, non-latching)
  reload                     reload settings from NVM (cmd 95, no reset)
  reset                      write NVM with unlock key — unit resets
  save                       alias for reset (WriteNVM)
  monitor [secs]             live table: voltage/temp/current/encoders/speed

Options:
  --port PORT      (default /dev/ttyO1)
  --baud BAUD      (default 460800)
  --addr ADDR      hex address (default 0x80)
  --no-latch       non-latching e-stop (default: latching)
  --debug          print raw TX/RX bytes

Config register bits (cmd 98/99):
  [1:0]  Control mode: 0=RC 1=Analog 2=Simple Serial 3=Packet Serial
  [4:2]  Battery LiPo cells (0=off)
  [7:5]  Baud: 0=2400 1=9600 2=19200 3=38400 4=57600 5=115200 6=230400 7=460800
  [8]    Flip switch
  [10:9] Packet address offset (0x80-0x87)
  [11]   Slave mode
  [12]   Relay mode
  [13]   Swap encoders
  [14]   Swap buttons
  [15]   Multi-unit mode
"""

import serial
import struct
import time
import sys
import argparse

PORT    = "/dev/ttyO1"
BAUD    = 460800
ADDRESS = 0x80
TIMEOUT = 0.15
DEBUG   = False

CMD_SET_PIN_FUNCTIONS   = 74
CMD_READ_PIN_FUNCTIONS  = 75
CMD_READ_ENC1           = 16
CMD_READ_ENC2           = 17
CMD_READ_SPEED1         = 18
CMD_READ_SPEED2         = 19
CMD_RESET_ENCODERS      = 20
CMD_READ_VERSION        = 21
CMD_READ_MAIN_BATT      = 24
CMD_READ_LOGIC_BATT     = 25
CMD_SET_M1_VELPID       = 28   # payload order: Kd, Kp, Ki, QPPS
CMD_SET_M2_VELPID       = 29
CMD_DRIVE_M1_SIGNED     = 32
CMD_DRIVE_M2_SIGNED     = 33
CMD_DRIVE_M1M2_SPEED    = 37
CMD_READ_MOTOR_CURRENTS = 49
CMD_READ_M1_VELPID      = 55
CMD_READ_M2_VELPID      = 56
CMD_READ_TEMP           = 82
CMD_READ_STATUS         = 90
CMD_SET_SERIAL_TIMEOUT  = 14   # payload: 1 byte, units of 100ms (0=disabled)
CMD_RELOAD_NVM          = 95   # reload from NVM, no reset
CMD_SET_CONFIG          = 98
CMD_GET_CONFIG          = 99
CMD_WRITE_NVM           = 94   # requires 0xE22EAB7A key, causes reset

# Config register baud bits [7:5]
BAUD_RATES = [2400, 9600, 19200, 38400, 57600, 115200, 230400, 460800]
BAUD_SHIFT = 5
BAUD_MASK  = 0x00E0

# S3/S4/S5 pin mode — bits [4:2] — NOTE: use cmd 74 for fine-grained control
# For coarse e-stop via config register:
CONFIG_ESTOP_LATCH   = 0b010
CONFIG_ESTOP_NOLATCH = 0b011
PIN_MODE_SHIFT       = 2


# Pin mode values for cmd 74 (S3/S4/S5/CTRL1/CTRL2)
PIN_MODE_NAMES = {
    0x00: "Default/Disabled",
    0x41: "E-Stop",
    0xC1: "E-Stop (Latching)",
    0x14: "Voltage Clamp",
    0x80: "Encoder Toggle",
    0x04: "Brake",
    0x11: "Stop M1",
    0x21: "Stop M2",
    0x84: "User Output",
    0x40: "Flip Switch",
}

PIN_MODE_OPTIONS = [
    (0x00, "Default/Disabled"),
    (0x41, "E-Stop (non-latching)"),
    (0xC1, "E-Stop (latching)"),
    (0x14, "Voltage Clamp"),
    (0x04, "Brake"),
    (0x11, "Stop M1"),
    (0x21, "Stop M2"),
    (0x80, "Encoder Toggle"),
    (0x84, "User Output"),
    (0x40, "Flip Switch"),
]
STATUS_NAMES = {
    0: "Normal", 1: "M1 OverCurrent", 2: "M2 OverCurrent",
    3: "E-Stop", 4: "Temp Error", 5: "Temp2 Error",
    6: "Main Batt High", 7: "Logic Batt High", 8: "Logic Batt Low",
    9: "Main Batt High Warn", 10: "Main Batt Low Warn",
    11: "Temp Warn", 12: "Temp2 Warn", 13: "M1 Home", 14: "M2 Home",
}

CTRL_MODES = {0: "RC", 1: "Analog", 2: "Simple Serial", 3: "Packet Serial"}

# ── CRC ───────────────────────────────────────────────────────────────────────

def crc16(data):
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if (crc & 0x8000) else (crc << 1)
            crc &= 0xFFFF
    return crc

# ── Serial ────────────────────────────────────────────────────────────────────

def open_serial(port, baud):
    try:
        ser = serial.Serial(port, baud, timeout=TIMEOUT)
        time.sleep(0.05)
        ser.reset_input_buffer()
        return ser
    except serial.SerialException as e:
        print("ERROR: %s" % e)
        sys.exit(1)

def send_cmd(ser, addr, cmd, payload=b""):
    packet = bytes([addr, cmd]) + payload
    full = packet + struct.pack(">H", crc16(packet))
    if DEBUG:
        print("  TX [%d]: %s" % (len(full), full.hex()))
    ser.write(full)

def read_n(ser, n):
    data = b""
    deadline = time.time() + TIMEOUT * 8
    while len(data) < n and time.time() < deadline:
        data += ser.read(n - len(data))
    if DEBUG and data:
        print("  RX [%d]: %s" % (len(data), data.hex()))
    return data

def check_ack(ser):
    b = read_n(ser, 1)
    ok = len(b) == 1 and b[0] == 0xFF
    if DEBUG:
        print("  ACK: %s" % ("OK" if ok else ("FAIL(%s)" % (b.hex() if b else "timeout"))))
    return ok

def send_recv(ser, addr, cmd, payload=b"", recv_n=0):
    send_cmd(ser, addr, cmd, payload)
    if recv_n == 0:
        return b"\xff" if check_ack(ser) else None
    raw = read_n(ser, recv_n + 2)
    if len(raw) < recv_n + 2:
        if DEBUG:
            print("  Short: got %d want %d" % (len(raw), recv_n + 2))
        return None
    data   = raw[:recv_n]
    rx_crc = struct.unpack(">H", raw[recv_n:recv_n+2])[0]
    if crc16(data) == rx_crc or crc16(bytes([addr, cmd]) + data) == rx_crc:
        return data
    if DEBUG:
        print("  CRC fail: rx=0x%04x" % rx_crc)
    return None

def do_write_nvm(ser, addr):
    """WriteNVM with unlock key. Unit resets ~1s after ACK. Waits 3s for it to come back."""
    payload = bytes([addr, CMD_WRITE_NVM]) + struct.pack(">I", 0xE22EAB7A)
    full = payload + struct.pack(">H", crc16(payload))
    ser.reset_input_buffer()
    ser.write(full)
    time.sleep(0.5)
    ack = ser.read(1)
    ok = ack and ack[0] == 0xFF
    if ok:
        print("  NVM written. Unit resetting — waiting 3s...")
        time.sleep(3.0)
        ser.reset_input_buffer()
    else:
        print("  NVM write: no ACK (%s)" % (ack.hex() if ack else "timeout"))
    return ok

# ── Commands ──────────────────────────────────────────────────────────────────

def cmd_probe(ser, addr):
    print("=== Probe addr=0x%02x port=%s baud=%d ===" % (addr, ser.port, ser.baudrate))
    ser.reset_input_buffer()
    packet = bytes([addr, CMD_READ_VERSION])
    full = packet + struct.pack(">H", crc16(packet))
    print("  Sending: %s" % full.hex())
    ser.write(full)
    time.sleep(0.25)
    raw = ser.read(64)
    if not raw:
        print("  No response. Check port, baud, address (try 0x80 or 0x01).")
    else:
        print("  Got %d bytes: %s" % (len(raw), raw.hex()))
        print("  ASCII: %s" % raw.decode("ascii", errors="replace").strip())

def cmd_status(ser, addr):
    print("=== RoboClaw Status ===")
    ser.reset_input_buffer()
    send_cmd(ser, addr, CMD_READ_VERSION)
    time.sleep(0.15)
    raw = ser.read(48)
    if raw:
        print("  Firmware : %s" % raw[:-2].decode("ascii", errors="replace").strip().strip("\x00"))
    else:
        print("  Firmware : (no response)")

    d = send_recv(ser, addr, CMD_READ_MAIN_BATT, recv_n=2)
    print("  Main bat : %s" % ("%.1f V" % (struct.unpack(">H", d)[0] / 10.0) if d else "--"))

    d = send_recv(ser, addr, CMD_READ_LOGIC_BATT, recv_n=2)
    print("  Logic bat: %s" % ("%.1f V" % (struct.unpack(">H", d)[0] / 10.0) if d else "--"))

    d = send_recv(ser, addr, CMD_READ_TEMP, recv_n=2)
    print("  Temp     : %s" % ("%.1f C" % (struct.unpack(">H", d)[0] / 10.0) if d else "--"))

    d = send_recv(ser, addr, CMD_READ_STATUS, recv_n=4)
    if d:
        flags = struct.unpack(">I", d)[0]
        print("  Status   : 0x%08x" % flags)
        if flags == 0:
            print("             OK")
        else:
            for bit, name in STATUS_NAMES.items():
                if flags & (1 << bit):
                    print("             [!] %s" % name)
    else:
        print("  Status   : --")

def cmd_encoders(ser, addr):
    print("=== Encoders ===")
    for ch, ce, cs in [
        (1, CMD_READ_ENC1, CMD_READ_SPEED1),
        (2, CMD_READ_ENC2, CMD_READ_SPEED2),
    ]:
        d = send_recv(ser, addr, ce, recv_n=5)
        if d:
            print("  M%d count : %12d  (status=0x%02x)" % (ch, struct.unpack(">i", d[:4])[0], d[4]))
        else:
            print("  M%d count : --" % ch)
        d = send_recv(ser, addr, cs, recv_n=5)
        if d:
            print("  M%d speed : %12d pps  (status=0x%02x)" % (ch, struct.unpack(">i", d[:4])[0], d[4]))
        else:
            print("  M%d speed : --" % ch)

def cmd_currents(ser, addr):
    print("=== Motor Currents ===")
    d = send_recv(ser, addr, CMD_READ_MOTOR_CURRENTS, recv_n=4)
    if d:
        print("  M1: %.2f A" % (struct.unpack(">H", d[0:2])[0] / 100.0))
        print("  M2: %.2f A" % (struct.unpack(">H", d[2:4])[0] / 100.0))
    else:
        print("  (no response)")

def cmd_read_pid(ser, addr, ch):
    cmd = CMD_READ_M1_VELPID if ch == 1 else CMD_READ_M2_VELPID
    d = send_recv(ser, addr, cmd, recv_n=16)
    if d:
        # Read back order from RoboClaw: P, I, D, QPPS
        p, i, dv, q = struct.unpack(">IIII", d)
        print("=== M%d Velocity PID ===" % ch)
        print("  Kp   : %s" % (p  / 65536.0))
        print("  Ki   : %s" % (i  / 65536.0))
        print("  Kd   : %s" % (dv / 65536.0))
        print("  QPPS : %d" % q)
    else:
        print("  (no response)")

def cmd_set_pid(ser, addr, ch, kp, ki, kd, qpps):
    cmd = CMD_SET_M1_VELPID if ch == 1 else CMD_SET_M2_VELPID
    # Write order: Kd, Kp, Ki, QPPS
    payload = struct.pack(">IIII",
        int(kd * 65536), int(kp * 65536), int(ki * 65536), int(qpps))
    r = send_recv(ser, addr, cmd, payload=payload)
    if r:
        print("  M%d PID set: Kp=%.4f Ki=%.4f Kd=%.4f QPPS=%d" % (ch, kp, ki, kd, qpps))
        do_write_nvm(ser, addr)
    else:
        print("  ERROR: set PID M%d failed" % ch)

def cmd_stop(ser, addr):
    print("=== Stop ===")
    r = send_recv(ser, addr, CMD_DRIVE_M1M2_SPEED, payload=struct.pack(">ii", 0, 0))
    if r:
        print("  Both motors stopped.")
    else:
        send_recv(ser, addr, CMD_DRIVE_M1_SIGNED, payload=struct.pack(">i", 0))
        send_recv(ser, addr, CMD_DRIVE_M2_SIGNED, payload=struct.pack(">i", 0))
        print("  Both motors stopped (fallback).")

def cmd_read_config(ser, addr):
    print("=== Config Register ===")
    d = send_recv(ser, addr, CMD_GET_CONFIG, recv_n=2)
    if not d:
        print("  (no response)")
        return None
    cfg = struct.unpack(">H", d)[0]
    baud_idx = (cfg & BAUD_MASK) >> BAUD_SHIFT
    baud_val = BAUD_RATES[baud_idx] if baud_idx < len(BAUD_RATES) else "?"
    print("  Raw      : 0x%04x  (%s)" % (cfg, format(cfg, "016b")))
    print("  Ctrl mode: %s" % CTRL_MODES.get(cfg & 0x03, str(cfg & 0x03)))
    print("  Baud     : %s (%d)" % (baud_val, baud_idx))
    print("  Addr     : 0x%02x" % (0x80 + ((cfg >> 9) & 0x07)))
    return cfg

def cmd_set_config_raw(ser, addr, cfg):
    print("=== Set Config Raw ===")
    print("  Writing : 0x%04x" % cfg)
    r = send_recv(ser, addr, CMD_SET_CONFIG, payload=struct.pack(">H", cfg))
    if r:
        print("  Written to RAM. Saving...")
        do_write_nvm(ser, addr)
    else:
        print("  ERROR: set config failed")

def cmd_set_baud(ser, addr, new_baud):
    print("=== Set Baud Rate ===")
    if new_baud not in BAUD_RATES:
        print("  ERROR: %d not a valid baud rate" % new_baud)
        print("  Valid: %s" % BAUD_RATES)
        return
    baud_idx = BAUD_RATES.index(new_baud)

    d = send_recv(ser, addr, CMD_GET_CONFIG, recv_n=2)
    if not d:
        print("  ERROR: could not read config")
        return
    cfg = struct.unpack(">H", d)[0]
    print("  Current config: 0x%04x  baud=%d" % (cfg, BAUD_RATES[(cfg & BAUD_MASK) >> BAUD_SHIFT]))

    cfg = (cfg & ~BAUD_MASK) | (baud_idx << BAUD_SHIFT)
    print("  New config    : 0x%04x  baud=%d" % (cfg, new_baud))

    r = send_recv(ser, addr, CMD_SET_CONFIG, payload=struct.pack(">H", cfg))
    if r:
        print("  Written. Saving (unit will reset)...")
        do_write_nvm(ser, addr)
        print("  Done. Reconnect at %d baud." % new_baud)
    else:
        print("  ERROR: set config failed")

def cmd_estop_config(ser, addr, latching=True):
    print("=== Configure E-Stop (S3/S4/S5) ===")
    d = send_recv(ser, addr, CMD_GET_CONFIG, recv_n=2)
    if not d:
        print("  ERROR: could not read config")
        return
    cfg = struct.unpack(">H", d)[0]
    print("  Current: 0x%04x" % cfg)
    mode = CONFIG_ESTOP_LATCH if latching else CONFIG_ESTOP_NOLATCH
    cfg  = (cfg & ~(0x07 << PIN_MODE_SHIFT)) | (mode << PIN_MODE_SHIFT)
    print("  New    : 0x%04x  (%s e-stop)" % (cfg, "latching" if latching else "non-latching"))
    r = send_recv(ser, addr, CMD_SET_CONFIG, payload=struct.pack(">H", cfg))
    if r:
        print("  Written. Saving...")
        do_write_nvm(ser, addr)
    else:
        print("  ERROR: set config failed")


def cmd_read_pins(ser, addr):
    print("=== Pin Functions (S3/S4/S5/CTRL1/CTRL2) ===")
    d = send_recv(ser, addr, CMD_READ_PIN_FUNCTIONS, recv_n=5)
    if not d:
        print("  (no response)")
        return None
    names = ["S3", "S4", "S5", "CTRL1", "CTRL2"]
    modes = list(d)
    for name, mode in zip(names, modes):
        label = PIN_MODE_NAMES.get(mode, "0x%02x" % mode)
        print("  %s : 0x%02x  %s" % (name, mode, label))
    return modes

def cmd_set_pins(ser, addr, pin_args):
    """
    Set pin functions. pin_args is a list of up to 5 hex values for S3,S4,S5,CTRL1,CTRL2.
    Missing values default to current settings.
    """
    print("=== Set Pin Functions ===")
    # Read current first
    d = send_recv(ser, addr, CMD_READ_PIN_FUNCTIONS, recv_n=5)
    if not d:
        print("  ERROR: could not read current pin modes")
        return
    modes = list(d)
    names = ["S3", "S4", "S5", "CTRL1", "CTRL2"]

    for i, val in enumerate(pin_args[:5]):
        try:
            modes[i] = int(val, 0)
        except ValueError:
            print("  ERROR: invalid value %s" % val)
            return

    for name, mode in zip(names, modes):
        label = PIN_MODE_NAMES.get(mode, "0x%02x" % mode)
        print("  %s -> 0x%02x  %s" % (name, mode, label))

    payload = bytes(modes)
    r = send_recv(ser, addr, CMD_SET_PIN_FUNCTIONS, payload=payload)
    if r:
        print("  Written. Saving...")
        do_write_nvm(ser, addr)
    else:
        print("  ERROR: set pin functions failed")

def cmd_estop_pin(ser, addr, pin="S4", latching=False):
    """Quick helper: configure a single pin as e-stop."""
    print("=== Configure %s as E-Stop ===" % pin)
    pin_map = {"S3": 0, "S4": 1, "S5": 2, "CTRL1": 3, "CTRL2": 4}
    if pin.upper() not in pin_map:
        print("  ERROR: unknown pin %s (use S3/S4/S5/CTRL1/CTRL2)" % pin)
        return
    idx = pin_map[pin.upper()]
    mode = 0xC1 if latching else 0x41

    d = send_recv(ser, addr, CMD_READ_PIN_FUNCTIONS, recv_n=5)
    if not d:
        print("  ERROR: could not read current modes")
        return
    modes = list(d)
    modes[idx] = mode
    label = "latching" if latching else "non-latching"
    print("  Setting %s to E-Stop (%s) = 0x%02x" % (pin.upper(), label, mode))
    payload = bytes(modes)
    r = send_recv(ser, addr, CMD_SET_PIN_FUNCTIONS, payload=payload)
    if r:
        print("  Written. Saving...")
        do_write_nvm(ser, addr)
    else:
        print("  ERROR: set pin functions failed")

def cmd_set_timeout(ser, addr, ms):
    """Set serial timeout. 0 = disabled. Value is in ms, stored as units of 100ms."""
    val = int(round(ms / 100))
    if val < 0 or val > 255:
        print("  ERROR: timeout must be 0-25500 ms")
        return
    r = send_recv(ser, addr, CMD_SET_SERIAL_TIMEOUT, payload=bytes([val]))
    if r is not None:
        print("  Serial timeout set to %d ms (%d x 100ms)" % (val * 100, val))
    else:
        print("  ERROR: set timeout failed")

def cmd_read_timeout(ser, addr):
    """Read current serial timeout setting."""
    d = send_recv(ser, addr, CMD_SET_SERIAL_TIMEOUT, recv_n=1)
    if d:
        val = d[0]
        print("  Serial timeout: %d ms (%d x 100ms)%s" % (
            val * 100, val, " (disabled)" if val == 0 else ""))
    else:
        print("  ERROR: read timeout failed")

def cmd_reload(ser, addr):
    print("=== Reload NVM (cmd 95) ===")
    r = send_recv(ser, addr, CMD_RELOAD_NVM)
    if r:
        print("  Settings reloaded from NVM (no reset).")
    else:
        print("  Failed or no ACK.")

def cmd_reset(ser, addr):
    print("=== WriteNVM / Reset ===")
    do_write_nvm(ser, addr)

def cmd_monitor(ser, addr, interval=1.0):
    print("=== Monitor (Ctrl-C to stop) ===")
    print("%6s  %6s  %6s  %6s  %6s  %12s  %12s  %8s  %8s" % (
        "Time", "Vbat", "Temp", "M1A", "M2A", "Enc1", "Enc2", "Spd1", "Spd2"))
    t0 = time.time()
    try:
        while True:
            t  = time.time() - t0
            d  = send_recv(ser, addr, CMD_READ_MAIN_BATT, recv_n=2)
            v  = struct.unpack(">H", d)[0] / 10.0 if d else float("nan")
            d  = send_recv(ser, addr, CMD_READ_TEMP, recv_n=2)
            tp = struct.unpack(">H", d)[0] / 10.0 if d else float("nan")
            d  = send_recv(ser, addr, CMD_READ_MOTOR_CURRENTS, recv_n=4)
            m1a = struct.unpack(">H", d[0:2])[0] / 100.0 if d else float("nan")
            m2a = struct.unpack(">H", d[2:4])[0] / 100.0 if d else float("nan")
            d  = send_recv(ser, addr, CMD_READ_ENC1,   recv_n=5)
            e1 = struct.unpack(">i", d[:4])[0] if d else 0
            d  = send_recv(ser, addr, CMD_READ_ENC2,   recv_n=5)
            e2 = struct.unpack(">i", d[:4])[0] if d else 0
            d  = send_recv(ser, addr, CMD_READ_SPEED1, recv_n=5)
            s1 = struct.unpack(">i", d[:4])[0] if d else 0
            d  = send_recv(ser, addr, CMD_READ_SPEED2, recv_n=5)
            s2 = struct.unpack(">i", d[:4])[0] if d else 0
            print("%6.1f  %6.1f  %6.1f  %6.2f  %6.2f  %12d  %12d  %8d  %8d" % (
                t, v, tp, m1a, m2a, e1, e2, s1, s2))
            time.sleep(interval)
    except KeyboardInterrupt:
        print("\nStopped.")

# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="RoboClaw packet serial utility")
    parser.add_argument("command", help="Command to run")
    parser.add_argument("args",    nargs="*", help="Command arguments")
    parser.add_argument("--port",     default="/dev/ttyO1")
    parser.add_argument("--baud",     type=int, default=460800)
    parser.add_argument("--addr",     type=lambda x: int(x, 0), default=0x80)
    parser.add_argument("--no-latch", action="store_true")
    parser.add_argument("--debug",    action="store_true")
    args = parser.parse_args()

    global DEBUG
    DEBUG = args.debug

    ser  = open_serial(args.port, args.baud)
    addr = args.addr
    cmd  = args.command.lower()

    if   cmd == "probe":        cmd_probe(ser, addr)
    elif cmd == "status":       cmd_status(ser, addr)
    elif cmd == "encoders":     cmd_encoders(ser, addr)
    elif cmd == "currents":     cmd_currents(ser, addr)
    elif cmd == "pid1":         cmd_read_pid(ser, addr, 1)
    elif cmd == "pid2":         cmd_read_pid(ser, addr, 2)
    elif cmd == "stop":         cmd_stop(ser, addr)
    elif cmd == "read_config":  cmd_read_config(ser, addr)
    elif cmd == "estop_config": cmd_estop_config(ser, addr, latching=not args.no_latch)
    elif cmd == "read_pins":    cmd_read_pins(ser, addr)
    elif cmd == "set_pins":
        if not args.args:
            print("Usage: set_pins S3hex S4hex S5hex CTRL1hex CTRL2hex"); sys.exit(1)
        cmd_set_pins(ser, addr, args.args)
    elif cmd == "estop_pin":
        pin = args.args[0].upper() if args.args else "S4"
        cmd_estop_pin(ser, addr, pin=pin, latching=args.no_latch == False and "--latch" in sys.argv)
    elif cmd == "reload":       cmd_reload(ser, addr)
    elif cmd in ("reset", "save"): cmd_reset(ser, addr)
    elif cmd == "set_config":
        if not args.args:
            print("Usage: set_config 0xHEX"); sys.exit(1)
        cmd_set_config_raw(ser, addr, int(args.args[0], 0))
    elif cmd == "set_timeout":
        if not args.args:
            print("Usage: set_timeout MS"); sys.exit(1)
        cmd_set_timeout(ser, addr, int(args.args[0]))
    elif cmd == "read_timeout":
        cmd_read_timeout(ser, addr)
    elif cmd == "set_baud":
        if not args.args:
            print("Usage: set_baud RATE"); sys.exit(1)
        cmd_set_baud(ser, addr, int(args.args[0]))
    elif cmd == "setpid1":
        if len(args.args) != 4:
            print("Usage: setpid1 Kp Ki Kd qpps"); sys.exit(1)
        kp, ki, kd, q = map(float, args.args)
        cmd_set_pid(ser, addr, 1, kp, ki, kd, int(q))
    elif cmd == "setpid2":
        if len(args.args) != 4:
            print("Usage: setpid2 Kp Ki Kd qpps"); sys.exit(1)
        kp, ki, kd, q = map(float, args.args)
        cmd_set_pid(ser, addr, 2, kp, ki, kd, int(q))
    elif cmd == "monitor":
        cmd_monitor(ser, addr, float(args.args[0]) if args.args else 1.0)
    else:
        print("Unknown command: %s" % cmd)
        print(__doc__)
        sys.exit(1)

    ser.close()

if __name__ == "__main__":
    main()