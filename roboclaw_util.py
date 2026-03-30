#!/usr/bin/env python3
"""
roboclaw_util.py — RoboClaw packet serial utility
BBB: /dev/ttyO1 @ 460800, address 0x80

Usage:
  python3 roboclaw_util.py <command> [args]

Commands:
  probe           -- raw comms test, dumps bytes
  status          -- firmware version, voltages, temps, error status
  encoders        -- encoder counts and speeds
  currents        -- motor currents
  pid1            -- read velocity PID for M1
  pid2            -- read velocity PID for M2
  setpid1 Kp Ki Kd qpps  -- set velocity PID for M1 and save
  setpid2 Kp Ki Kd qpps  -- set velocity PID for M2 and save
  stop            -- zero both motors
  estop_config    -- configure S3/S4/S5 as e-stop inputs and save
  read_config     -- dump and decode config register
  save            -- write settings to EEPROM
  monitor [secs]  -- live table: voltage/temp/current/encoders/speed
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

CMD_READ_ENC1           = 16
CMD_READ_ENC2           = 17
CMD_READ_SPEED1         = 18
CMD_READ_SPEED2         = 19
CMD_RESET_ENCODERS      = 20
CMD_READ_VERSION        = 21
CMD_READ_MAIN_BATT      = 24
CMD_READ_LOGIC_BATT     = 25
CMD_SET_M1_VELPID       = 28
CMD_SET_M2_VELPID       = 29
CMD_DRIVE_M1_SIGNED     = 32
CMD_DRIVE_M2_SIGNED     = 33
CMD_DRIVE_M1M2_SPEED    = 37
CMD_READ_MOTOR_CURRENTS = 49
CMD_READ_M1_VELPID      = 55
CMD_READ_M2_VELPID      = 56
CMD_READ_TEMP           = 82
CMD_READ_STATUS         = 90
CMD_WRITE_SETTINGS      = 94
CMD_SET_CONFIG          = 98
CMD_GET_CONFIG          = 99

PIN_MODE_SHIFT       = 2
CONFIG_ESTOP_LATCH   = 0b010
CONFIG_ESTOP_NOLATCH = 0b011


def crc16(data):
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if (crc & 0x8000) else (crc << 1)
            crc &= 0xFFFF
    return crc


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
        print("  TX [%d]: %s" % (len(full), full.hex(" ")))
    ser.write(full)


def read_n(ser, n):
    data = b""
    deadline = time.time() + TIMEOUT * 8
    while len(data) < n and time.time() < deadline:
        data += ser.read(n - len(data))
    if DEBUG and data:
        print("  RX [%d]: %s" % (len(data), data.hex(" ")))
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
    if crc16(data) == rx_crc:
        return data
    if crc16(bytes([addr, cmd]) + data) == rx_crc:
        return data
    if DEBUG:
        print("  CRC fail: rx=0x%04x data=0x%04x hdr=0x%04x" % (
            rx_crc, crc16(data), crc16(bytes([addr, cmd]) + data)))
    return None


def cmd_probe(ser, addr):
    print("=== Probe addr=0x%02x port=%s baud=%d ===" % (addr, ser.port, ser.baudrate))
    ser.reset_input_buffer()
    packet = bytes([addr, CMD_READ_VERSION])
    full = packet + struct.pack(">H", crc16(packet))
    print("  Sending: %s" % full.hex(" "))
    ser.write(full)
    time.sleep(0.25)
    raw = ser.read(64)
    if not raw:
        print("  No response. Check port, baud, address (try 0x80 or 0x01).")
    else:
        print("  Got %d bytes: %s" % (len(raw), raw.hex(" ")))
        print("  ASCII: %s" % raw.decode("ascii", errors="replace").strip())


def cmd_status(ser, addr):
    print("=== RoboClaw Status ===")

    ser.reset_input_buffer()
    send_cmd(ser, addr, CMD_READ_VERSION)
    time.sleep(0.15)
    raw = ser.read(48)
    if raw:
        version = raw[:-2].decode("ascii", errors="replace").strip().strip("\x00")
        print("  Firmware : %s" % version)
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
            names = {
                0: "Normal", 1: "M1 OverCurrent", 2: "M2 OverCurrent",
                3: "E-Stop", 4: "Temp Error", 5: "Temp2 Error",
                6: "Main Batt High", 7: "Logic Batt High", 8: "Logic Batt Low",
                9: "Main Batt High Warn", 10: "Main Batt Low Warn",
                11: "Temp Warn", 12: "Temp2 Warn", 13: "M1 Home", 14: "M2 Home",
            }
            for bit, name in names.items():
                if flags & (1 << bit):
                    print("             [!] %s" % name)
    else:
        print("  Status   : --")


def cmd_encoders(ser, addr):
    print("=== Encoders ===")
    for ch, ce, cs in [(1, CMD_READ_ENC1, CMD_READ_SPEED1), (2, CMD_READ_ENC2, CMD_READ_SPEED2)]:
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
        p, i, v, q = struct.unpack(">IIII", d)
        print("=== M%d Velocity PID ===" % ch)
        print("  Kp   : %s" % (p / 65536.0))
        print("  Ki   : %s" % (i / 65536.0))
        print("  Kd   : %s" % (v / 65536.0))
        print("  QPPS : %d" % q)
    else:
        print("  (no response)")


def cmd_set_pid(ser, addr, ch, kp, ki, kd, qpps):
    cmd = CMD_SET_M1_VELPID if ch == 1 else CMD_SET_M2_VELPID
    payload = struct.pack(">IIII", int(kp*65536), int(ki*65536), int(kd*65536), int(qpps))
    r = send_recv(ser, addr, cmd, payload=payload)
    if r:
        print("  M%d PID set OK" % ch)
        cmd_save(ser, addr)
    else:
        print("  ERROR: set PID failed")


def cmd_stop(ser, addr):
    print("=== Stop ===")
    r = send_recv(ser, addr, CMD_DRIVE_M1M2_SPEED, payload=struct.pack(">ii", 0, 0))
    if r:
        print("  Stopped.")
    else:
        send_recv(ser, addr, CMD_DRIVE_M1_SIGNED, payload=struct.pack(">i", 0))
        send_recv(ser, addr, CMD_DRIVE_M2_SIGNED, payload=struct.pack(">i", 0))
        print("  Stopped (fallback).")


def cmd_read_config(ser, addr):
    print("=== Config ===")
    d = send_recv(ser, addr, CMD_GET_CONFIG, recv_n=2)
    if not d:
        print("  (no response)")
        return None
    cfg = struct.unpack(">H", d)[0]
    print("  Raw      : 0x%04x  (%016b)" % (cfg, cfg))
    modes = {0: "Packet Serial", 1: "Analog", 2: "RC", 3: "Simple Serial"}
    print("  Ctrl mode: %s" % modes.get(cfg & 0x07, str(cfg & 0x07)))
    pm = (cfg >> PIN_MODE_SHIFT) & 0x07
    pin_modes = {0: "Default", 1: "Home/Limit", 2: "E-Stop (latch)", 3: "E-Stop (no-latch)", 4: "Vclamp out"}
    print("  Pin mode : %s" % pin_modes.get(pm, "0b%03b" % pm))
    return cfg


def cmd_estop_config(ser, addr, latching=True):
    print("=== Configure E-Stop ===")
    d = send_recv(ser, addr, CMD_GET_CONFIG, recv_n=2)
    if not d:
        print("  ERROR: could not read config")
        return
    cfg = struct.unpack(">H", d)[0]
    print("  Current: 0x%04x" % cfg)
    mode = CONFIG_ESTOP_LATCH if latching else CONFIG_ESTOP_NOLATCH
    cfg = (cfg & ~(0x07 << PIN_MODE_SHIFT)) | (mode << PIN_MODE_SHIFT)
    print("  New    : 0x%04x  (%s)" % (cfg, "latching" if latching else "non-latching"))
    r = send_recv(ser, addr, CMD_SET_CONFIG, payload=struct.pack(">H", cfg))
    if r:
        print("  Written.")
        cmd_save(ser, addr)
    else:
        print("  ERROR: set config failed")


def cmd_save(ser, addr):
    r = send_recv(ser, addr, CMD_WRITE_SETTINGS)
    print("  %s" % ("Saved to EEPROM." if r else "Save failed."))


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


def main():
    parser = argparse.ArgumentParser(description="RoboClaw utility")
    parser.add_argument("command")
    parser.add_argument("args", nargs="*")
    parser.add_argument("--port",     default=PORT)
    parser.add_argument("--baud",     type=int, default=BAUD)
    parser.add_argument("--addr",     type=lambda x: int(x, 0), default=ADDRESS)
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
    elif cmd == "estop_config": cmd_estop_config(ser, addr, latching=not args.no_latch)
    elif cmd == "read_config":  cmd_read_config(ser, addr)
    elif cmd == "save":         cmd_save(ser, addr)
    elif cmd == "setpid1":
        if len(args.args) != 4: print("Usage: setpid1 Kp Ki Kd qpps"); sys.exit(1)
        kp, ki, kd, q = map(float, args.args)
        cmd_set_pid(ser, addr, 1, kp, ki, kd, int(q))
    elif cmd == "setpid2":
        if len(args.args) != 4: print("Usage: setpid2 Kp Ki Kd qpps"); sys.exit(1)
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
