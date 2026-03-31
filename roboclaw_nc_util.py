#!/usr/bin/env python3
"""
roboclaw_nc_util.py — RoboClaw 2x7A ncurses dashboard
Part of roboclaw-util

Usage:
  python3 roboclaw_nc_util.py [--port /dev/ttyO1] [--baud 460800] [--addr 0x80]

Keys:
  s     — stop both motors
  r     — reset encoders
  b     — set baud rate
  p     — set PID params (M1 and M2)
  q     — quit
  d     — toggle debug overlay
"""

import curses
import serial
import struct
import time
import sys
import argparse
import threading
import collections

TIMEOUT = 0.15
POLL_HZ = 10

CMD_M1_DUTY             = 0
CMD_M2_DUTY             = 4
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
CMD_WRITE_NVM           = 94
CMD_SET_BAUD            = 169

BAUD_RATES = [2400, 9600, 19200, 38400, 57600, 115200, 230400, 460800]

STATUS_NAMES = {
    1:  "M1 OVERCURRENT",
    2:  "M2 OVERCURRENT",
    3:  "E-STOP",
    4:  "TEMP ERROR",
    6:  "BATT HIGH",
    8:  "LOGIC LOW",
    10: "BATT LOW",
    11: "TEMP WARN",
}

# ── CRC + serial ──────────────────────────────────────────────────────────────

def crc16(data):
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if (crc & 0x8000) else (crc << 1)
            crc &= 0xFFFF
    return crc

def send_cmd(ser, addr, cmd, payload=b""):
    packet = bytes([addr, cmd]) + payload
    ser.write(packet + struct.pack(">H", crc16(packet)))

def read_n(ser, n):
    data = b""
    deadline = time.time() + TIMEOUT * 6
    while len(data) < n and time.time() < deadline:
        data += ser.read(n - len(data))
    return data

def send_recv(ser, addr, cmd, payload=b"", recv_n=0):
    send_cmd(ser, addr, cmd, payload)
    if recv_n == 0:
        b = read_n(ser, 1)
        return b"\xff" if (b and b[0] == 0xFF) else None
    raw = read_n(ser, recv_n + 2)
    if len(raw) < recv_n + 2:
        return None
    data   = raw[:recv_n]
    rx_crc = struct.unpack(">H", raw[recv_n:recv_n+2])[0]
    if crc16(data) == rx_crc or crc16(bytes([addr, cmd]) + data) == rx_crc:
        return data
    return None

def save_nvm(ser, addr):
    """Save with plain cmd 94 (no unlock key — safer for v4.2.8)."""
    packet = bytes([addr, CMD_WRITE_NVM])
    ser.write(packet + struct.pack(">H", crc16(packet)))
    time.sleep(0.3)
    ack = ser.read(1)
    return ack and ack[0] == 0xFF

# ── Telemetry state ───────────────────────────────────────────────────────────

class State:
    def __init__(self):
        self.lock        = threading.Lock()
        self.connected   = False
        self.firmware    = "..."
        self.vbat        = 0.0
        self.vlogic      = 0.0
        self.temp        = 0.0
        self.status_flags = 0
        self.m1_current  = 0.0
        self.m2_current  = 0.0
        self.m1_speed    = 0
        self.m2_speed    = 0
        self.m1_enc      = 0
        self.m2_enc      = 0
        self.m1_kp = self.m1_ki = self.m1_kd = 0.0
        self.m1_qpps     = 0
        self.m2_kp = self.m2_ki = self.m2_kd = 0.0
        self.m2_qpps     = 0
        self.poll_count  = 0
        self.errors      = 0
        self.log         = collections.deque(maxlen=6)
        self.start_time  = time.time()

    def log_msg(self, msg):
        ts = time.strftime("%H:%M:%S")
        with self.lock:
            self.log.append("[%s] %s" % (ts, msg))

# ── Poller thread ─────────────────────────────────────────────────────────────

def poller(ser, addr, state, stop_event):
    ser.reset_input_buffer()
    send_cmd(ser, addr, CMD_READ_VERSION)
    time.sleep(0.15)
    raw = ser.read(48)
    if raw:
        fw = raw[:-2].decode("ascii", errors="replace").strip().strip("\x00")
        with state.lock:
            state.firmware  = fw
            state.connected = True
        state.log_msg("Connected: %s" % fw)
    else:
        state.log_msg("WARNING: no firmware response")

    for ch in (1, 2):
        cmd = CMD_READ_M1_VELPID if ch == 1 else CMD_READ_M2_VELPID
        d = send_recv(ser, addr, cmd, recv_n=16)
        if d:
            p, i, dv, q = struct.unpack(">IIII", d)
            with state.lock:
                if ch == 1:
                    state.m1_kp   = p  / 65536.0
                    state.m1_ki   = i  / 65536.0
                    state.m1_kd   = dv / 65536.0
                    state.m1_qpps = q
                else:
                    state.m2_kp   = p  / 65536.0
                    state.m2_ki   = i  / 65536.0
                    state.m2_kd   = dv / 65536.0
                    state.m2_qpps = q

    interval = 1.0 / POLL_HZ
    while not stop_event.is_set():
        t0 = time.time()
        try:
            d = send_recv(ser, addr, CMD_READ_MAIN_BATT, recv_n=2)
            if d:
                with state.lock: state.vbat = struct.unpack(">H", d)[0] / 10.0

            d = send_recv(ser, addr, CMD_READ_LOGIC_BATT, recv_n=2)
            if d:
                with state.lock: state.vlogic = struct.unpack(">H", d)[0] / 10.0

            d = send_recv(ser, addr, CMD_READ_TEMP, recv_n=2)
            if d:
                with state.lock: state.temp = struct.unpack(">H", d)[0] / 10.0

            d = send_recv(ser, addr, CMD_READ_STATUS, recv_n=4)
            if d:
                with state.lock: state.status_flags = struct.unpack(">I", d)[0]

            d = send_recv(ser, addr, CMD_READ_MOTOR_CURRENTS, recv_n=4)
            if d:
                with state.lock:
                    state.m1_current = struct.unpack(">H", d[0:2])[0] / 100.0
                    state.m2_current = struct.unpack(">H", d[2:4])[0] / 100.0

            d = send_recv(ser, addr, CMD_READ_SPEED1, recv_n=5)
            if d:
                with state.lock: state.m1_speed = struct.unpack(">i", d[:4])[0]

            d = send_recv(ser, addr, CMD_READ_SPEED2, recv_n=5)
            if d:
                with state.lock: state.m2_speed = struct.unpack(">i", d[:4])[0]

            d = send_recv(ser, addr, CMD_READ_ENC1, recv_n=5)
            if d:
                with state.lock: state.m1_enc = struct.unpack(">i", d[:4])[0]

            d = send_recv(ser, addr, CMD_READ_ENC2, recv_n=5)
            if d:
                with state.lock: state.m2_enc = struct.unpack(">i", d[:4])[0]

            with state.lock:
                state.poll_count += 1
                state.connected   = True

        except Exception as ex:
            with state.lock:
                state.errors   += 1
                state.connected = False
            state.log_msg("Poll error: %s" % str(ex))

        stop_event.wait(max(0, interval - (time.time() - t0)))


# ── Drawing helpers ───────────────────────────────────────────────────────────

def bar(val, maxval, width, full="█", empty="░"):
    if maxval <= 0: maxval = 1
    filled = max(0, min(int(round(min(val, maxval) / maxval * width)), width))
    return full * filled + empty * (width - filled)

def safe_add(win, y, x, s, attr=0):
    h, w = win.getmaxyx()
    if y < 0 or y >= h or x < 0 or x >= w: return
    max_len = w - x - 1
    if max_len <= 0: return
    try: win.addstr(y, x, s[:max_len], attr)
    except curses.error: pass

def box_title(win, y, x, width, title, ba, ta):
    safe_add(win, y, x,         "┌" + "─" * (width-2) + "┐", ba)
    tx = x + (width - len(title) - 2) // 2
    safe_add(win, y, tx, " %s " % title, ta)

def box_bottom(win, y, x, width, attr):
    safe_add(win, y, x, "└" + "─" * (width-2) + "┘", attr)

def box_side(win, y, x, width, attr):
    safe_add(win, y, x,         "│", attr)
    safe_add(win, y, x+width-1, "│", attr)


# ── Interactive input (drops out of curses briefly) ──────────────────────────

def curses_input(stdscr, prompt):
    """Show a prompt at bottom of screen, read a line of input."""
    h, w = stdscr.getmaxyx()
    stdscr.nodelay(False)
    curses.echo()
    curses.curs_set(1)
    safe_add(stdscr, h-1, 0, " " * (w-1), curses.color_pair(6))
    safe_add(stdscr, h-1, 0, prompt, curses.color_pair(6) | curses.A_BOLD)
    stdscr.refresh()
    try:
        val = stdscr.getstr(h-1, len(prompt), 40).decode("utf-8").strip()
    except Exception:
        val = ""
    curses.noecho()
    curses.curs_set(0)
    stdscr.nodelay(True)
    return val

def curses_menu(stdscr, title, options):
    """Show a numbered menu, return selected index or -1."""
    h, w = stdscr.getmaxyx()
    stdscr.nodelay(False)
    curses.curs_set(0)
    # Draw menu box in center
    mw = max(len(title), max(len(o) for o in options)) + 6
    mh = len(options) + 4
    my = (h - mh) // 2
    mx = (w - mw) // 2
    ba = curses.color_pair(1) | curses.A_BOLD
    ta = curses.color_pair(6) | curses.A_BOLD

    for i in range(mh):
        safe_add(stdscr, my+i, mx, " " * mw, curses.color_pair(5))

    box_title(stdscr, my, mx, mw, title, ba, ta)
    for i, opt in enumerate(options):
        safe_add(stdscr, my+2+i, mx+2, "[%d] %s" % (i, opt), curses.color_pair(5))
    box_bottom(stdscr, my+mh-1, mx, mw, ba)
    stdscr.refresh()

    while True:
        k = stdscr.getch()
        if k == 27 or k == ord('q'):   # ESC or q = cancel
            stdscr.nodelay(True)
            return -1
        if ord('0') <= k <= ord('0') + len(options) - 1:
            stdscr.nodelay(True)
            return k - ord('0')


# ── Config actions ────────────────────────────────────────────────────────────

def action_set_baud(stdscr, ser, addr, state, cur_baud):
    opts = ["%d baud" % b for b in BAUD_RATES]
    idx = curses_menu(stdscr, "SET BAUD RATE", opts)
    if idx < 0:
        state.log_msg("Baud change cancelled")
        return cur_baud

    new_baud = BAUD_RATES[idx]
    # cmd 169: payload is baud index 0-7
    r = send_recv(ser, addr, CMD_SET_BAUD, payload=bytes([idx]))
    if r:
        state.log_msg("Baud set to %d — saving..." % new_baud)
        save_nvm(ser, addr)
        state.log_msg("Saved. Reconnecting at %d..." % new_baud)
        time.sleep(1.0)
        ser.baudrate = new_baud
        time.sleep(0.1)
        ser.reset_input_buffer()
        return new_baud
    else:
        state.log_msg("ERROR: baud set failed")
        return cur_baud

def action_set_pid(stdscr, ser, addr, state):
    # Get values for M1
    state.log_msg("Setting PID — enter values (blank=keep current)")

    with state.lock:
        m1kp = state.m1_kp; m1ki = state.m1_ki
        m1kd = state.m1_kd; m1q  = state.m1_qpps
        m2kp = state.m2_kp; m2ki = state.m2_ki
        m2kd = state.m2_kd; m2q  = state.m2_qpps

    def get_float(prompt, current):
        s = curses_input(stdscr, "%s [%.4f]: " % (prompt, current))
        if s == "": return current
        try: return float(s)
        except ValueError: return current

    def get_int(prompt, current):
        s = curses_input(stdscr, "%s [%d]: " % (prompt, current))
        if s == "": return current
        try: return int(s)
        except ValueError: return current

    m1kp = get_float("M1 Kp", m1kp)
    m1ki = get_float("M1 Ki", m1ki)
    m1kd = get_float("M1 Kd", m1kd)
    m1q  = get_int  ("M1 QPPS", m1q)

    m2kp = get_float("M2 Kp", m2kp)
    m2ki = get_float("M2 Ki", m2ki)
    m2kd = get_float("M2 Kd", m2kd)
    m2q  = get_int  ("M2 QPPS", m2q)

    # Write M1 — packet order: Kd, Kp, Ki, QPPS
    payload = struct.pack(">IIII",
        int(m1kd*65536), int(m1kp*65536), int(m1ki*65536), m1q)
    r1 = send_recv(ser, addr, CMD_SET_M1_VELPID, payload=payload)

    payload = struct.pack(">IIII",
        int(m2kd*65536), int(m2kp*65536), int(m2ki*65536), m2q)
    r2 = send_recv(ser, addr, CMD_SET_M2_VELPID, payload=payload)

    if r1 and r2:
        state.log_msg("PID set OK — saving...")
        save_nvm(ser, addr)
        state.log_msg("Saved. M1 P%.2f I%.2f D%.2f Q%d" % (m1kp, m1ki, m1kd, m1q))
        # Refresh PID display
        with state.lock:
            state.m1_kp=m1kp; state.m1_ki=m1ki
            state.m1_kd=m1kd; state.m1_qpps=m1q
            state.m2_kp=m2kp; state.m2_ki=m2ki
            state.m2_kd=m2kd; state.m2_qpps=m2q
    else:
        state.log_msg("ERROR: PID set failed (M1=%s M2=%s)" % (
            "OK" if r1 else "FAIL", "OK" if r2 else "FAIL"))


# ── Draw ──────────────────────────────────────────────────────────────────────

def draw(stdscr, state, show_debug, cur_baud):
    curses.curs_set(0)
    stdscr.nodelay(True)

    curses.init_pair(1, curses.COLOR_CYAN,    curses.COLOR_BLACK)
    curses.init_pair(2, curses.COLOR_GREEN,   curses.COLOR_BLACK)
    curses.init_pair(3, curses.COLOR_YELLOW,  curses.COLOR_BLACK)
    curses.init_pair(4, curses.COLOR_RED,     curses.COLOR_BLACK)
    curses.init_pair(5, curses.COLOR_WHITE,   curses.COLOR_BLACK)
    curses.init_pair(6, curses.COLOR_BLACK,   curses.COLOR_CYAN)
    curses.init_pair(7, curses.COLOR_MAGENTA, curses.COLOR_BLACK)

    C_BORDER = curses.color_pair(1)
    C_OK     = curses.color_pair(2)
    C_WARN   = curses.color_pair(3)
    C_ERR    = curses.color_pair(4)
    C_NORMAL = curses.color_pair(5)
    C_HILITE = curses.color_pair(6)
    C_ACCENT = curses.color_pair(7)
    C_DIM    = curses.color_pair(5) | curses.A_DIM
    C_HEAD   = curses.color_pair(1) | curses.A_BOLD

    last_draw = 0.0

    while True:
        key = stdscr.getch()
        if   key == ord('q'): return 'quit',   cur_baud
        elif key == ord('s'): return 'stop',   cur_baud
        elif key == ord('r'): return 'reset',  cur_baud
        elif key == ord('b'): return 'baud',   cur_baud
        elif key == ord('p'): return 'pid',    cur_baud
        elif key == ord('d'): return 'debug',  cur_baud

        now = time.time()
        if now - last_draw < 0.08:
            time.sleep(0.02)
            continue
        last_draw = now

        h, w = stdscr.getmaxyx()
        if h < 24 or w < 80:
            stdscr.clear()
            safe_add(stdscr, 0, 0, "Terminal too small (need 80x24+)", C_ERR)
            stdscr.refresh()
            continue

        stdscr.erase()

        with state.lock:
            s        = state.__dict__.copy()
            log_lines = list(state.log)

        uptime = int(now - s["start_time"])
        up_str = "%02d:%02d:%02d" % (uptime//3600, (uptime%3600)//60, uptime%60)

        # Header
        safe_add(stdscr, 0, 0, " " * w, C_HILITE)
        safe_add(stdscr, 0, 2, "RoboClaw Monitor", C_HILITE | curses.A_BOLD)
        safe_add(stdscr, 0, 20, "2x7A · Packet Serial", C_HILITE)
        conn = "● CONNECTED" if s["connected"] else "○ OFFLINE"
        safe_add(stdscr, 0, w - len(conn) - len(up_str) - 4, conn, C_HILITE | curses.A_BOLD)
        safe_add(stdscr, 0, w - len(up_str) - 2, up_str, C_HILITE)

        left_w  = 22
        right_w = 22
        mid_w   = w - left_w - right_w - 2
        mid_x   = left_w + 1
        right_x = w - right_w
        row     = 1

        # ── Left: SYSTEM ──────────────────────────────────────────────────────
        box_title(stdscr, row, 0, left_w, "SYSTEM", C_BORDER, C_HEAD)
        for i in range(1, 8): box_side(stdscr, row+i, 0, left_w, C_BORDER)
        box_bottom(stdscr, row+8, 0, left_w, C_BORDER)

        vbat   = s["vbat"]
        v_attr = C_OK if vbat > 10.5 else (C_WARN if vbat > 9.6 else C_ERR)
        safe_add(stdscr, row+1, 2, "VBAT", C_DIM)
        safe_add(stdscr, row+1, 7, "%5.2fV" % vbat, v_attr | curses.A_BOLD)
        safe_add(stdscr, row+2, 2, bar(vbat-9.0, 3.6, left_w-4)[:left_w-4], v_attr)

        temp   = s["temp"]
        t_attr = C_OK if temp < 60 else (C_WARN if temp < 85 else C_ERR)
        safe_add(stdscr, row+3, 2, "TEMP", C_DIM)
        safe_add(stdscr, row+3, 7, "%5.1fC" % temp, t_attr | curses.A_BOLD)
        safe_add(stdscr, row+4, 2, bar(temp, 100.0, left_w-4)[:left_w-4], t_attr)

        safe_add(stdscr, row+5, 2, "LOGIC", C_DIM)
        safe_add(stdscr, row+5, 8, "%4.1fV" % s["vlogic"], C_NORMAL)

        safe_add(stdscr, row+6, 2, "BAUD ", C_DIM)
        safe_add(stdscr, row+6, 8, "%d" % cur_baud, C_NORMAL)

        err_attr = C_ERR if s["errors"] > 0 else C_DIM
        safe_add(stdscr, row+7, 2,  "POLL %5d" % s["poll_count"], C_DIM)
        safe_add(stdscr, row+7, 14, "ERR", C_DIM)
        safe_add(stdscr, row+7, 18, "%3d" % s["errors"], err_attr)

        # ── Left: STATUS ──────────────────────────────────────────────────────
        fr = row + 10
        box_title(stdscr, fr, 0, left_w, "STATUS", C_BORDER, C_HEAD)
        flags  = s["status_flags"]
        active = [(b, n) for b, n in STATUS_NAMES.items() if flags & (1 << b)]
        if not active:
            box_side(stdscr, fr+1, 0, left_w, C_BORDER)
            safe_add(stdscr, fr+1, 2, "ALL CLEAR", C_OK | curses.A_BOLD)
            for i in range(2, 5): box_side(stdscr, fr+i, 0, left_w, C_BORDER)
        else:
            for i in range(4):
                box_side(stdscr, fr+1+i, 0, left_w, C_BORDER)
                if i < len(active):
                    bit, name = active[i]
                    fa = C_ERR if bit in (3,4,6,8) else C_WARN
                    safe_add(stdscr, fr+1+i, 2, "▲ %-16s" % name[:16], fa | curses.A_BOLD)
        box_bottom(stdscr, fr+5, 0, left_w, C_BORDER)

        # ── Right: CONTROLS ───────────────────────────────────────────────────
        box_title(stdscr, row, right_x, right_w, "CONTROLS", C_BORDER, C_HEAD)
        for key_ch, label in [("s","STOP MOTORS"),("r","RESET ENCODERS"),
                               ("b","SET BAUD"),  ("p","SET PID"),
                               ("d","DEBUG"),     ("q","QUIT")]:
            i = [("s","STOP MOTORS"),("r","RESET ENCODERS"),
                 ("b","SET BAUD"),  ("p","SET PID"),
                 ("d","DEBUG"),     ("q","QUIT")].index((key_ch, label))
            box_side(stdscr, row+1+i, right_x, right_w, C_BORDER)
            safe_add(stdscr, row+1+i, right_x+2, "[", C_DIM)
            safe_add(stdscr, row+1+i, right_x+3, key_ch, C_ACCENT | curses.A_BOLD)
            safe_add(stdscr, row+1+i, right_x+4, "] " + label, C_NORMAL)
        box_bottom(stdscr, row+7, right_x, right_w, C_BORDER)

        # ── Right: FIRMWARE ───────────────────────────────────────────────────
        fw_r = row + 9
        box_title(stdscr, fw_r, right_x, right_w, "FIRMWARE", C_BORDER, C_HEAD)
        for i in range(1, 5): box_side(stdscr, fw_r+i, right_x, right_w, C_BORDER)
        box_bottom(stdscr, fw_r+5, right_x, right_w, C_BORDER)
        safe_add(stdscr, fw_r+1, right_x+2, s["firmware"][:right_w-4], C_OK)
        safe_add(stdscr, fw_r+2, right_x+2, ("ADDR 0x%02X" % addr)[:right_w-4], C_DIM)
        safe_add(stdscr, fw_r+3, right_x+2, port[:right_w-4], C_DIM)

        # ── Center: Motors ────────────────────────────────────────────────────
        motor_h = 8
        for mi, (label, speed, current, enc, kp, ki, kd, qpps) in enumerate([
            ("MOTOR 1", s["m1_speed"], s["m1_current"], s["m1_enc"],
             s["m1_kp"], s["m1_ki"], s["m1_kd"], s["m1_qpps"]),
            ("MOTOR 2", s["m2_speed"], s["m2_current"], s["m2_enc"],
             s["m2_kp"], s["m2_ki"], s["m2_kd"], s["m2_qpps"]),
        ]):
            mr = row + mi * (motor_h + 1)
            box_title(stdscr, mr, mid_x, mid_w, label, C_BORDER, C_HEAD)
            for i in range(1, motor_h): box_side(stdscr, mr+i, mid_x, mid_w, C_BORDER)
            box_bottom(stdscr, mr+motor_h, mid_x, mid_w, C_BORDER)

            iw = mid_w - 4
            cx = mid_x + 2

            spd_attr = C_OK if abs(speed) < qpps * 0.9 else C_WARN
            spd_dir  = "►" if speed > 0 else ("◄" if speed < 0 else "■")
            safe_add(stdscr, mr+1, cx, "SPD", C_DIM)
            safe_add(stdscr, mr+1, cx+4, "%s %6d pps" % (spd_dir, speed), spd_attr | curses.A_BOLD)
            safe_add(stdscr, mr+2, cx, bar(abs(speed), max(qpps,1), iw-2)[:iw-2], spd_attr)

            c_attr = C_OK if current < 4.0 else (C_WARN if current < 6.5 else C_ERR)
            safe_add(stdscr, mr+3, cx, "AMP", C_DIM)
            safe_add(stdscr, mr+3, cx+4, "%5.2f A" % current, c_attr | curses.A_BOLD)
            safe_add(stdscr, mr+4, cx, bar(current, 7.5, iw-2)[:iw-2], c_attr)

            safe_add(stdscr, mr+5, cx, "ENC", C_DIM)
            safe_add(stdscr, mr+5, cx+4, "%12d" % enc, C_NORMAL | curses.A_BOLD)

            pid_str = "P%-5.2f I%-5.2f D%-5.2f  QPPS %d" % (kp, ki, kd, qpps)
            safe_add(stdscr, mr+6, cx, pid_str[:iw], C_DIM)

        # ── Log ───────────────────────────────────────────────────────────────
        log_start = h - len(log_lines) - 2
        box_title(stdscr, log_start-1, 0, w, "LOG", C_BORDER, C_HEAD)
        for i, line in enumerate(log_lines):
            age  = len(log_lines) - i
            attr = C_NORMAL if age <= 2 else C_DIM
            safe_add(stdscr, log_start+i, 2, line[:w-4], attr)

        stdscr.refresh()


# ── TUI main loop ─────────────────────────────────────────────────────────────

def tui_main(stdscr, ser, addr, state, init_baud):
    show_debug = [False]
    stop_event = threading.Event()
    cur_baud   = [init_baud]

    poll_thread = threading.Thread(
        target=poller, args=(ser, addr, state, stop_event), daemon=True)
    poll_thread.start()

    try:
        while True:
            action, cur_baud[0] = draw(stdscr, state, show_debug, cur_baud[0])
            if action == 'quit':
                break
            elif action == 'stop':
                send_recv(ser, addr, CMD_DRIVE_M1M2_SPEED,
                          payload=struct.pack(">ii", 0, 0))
                state.log_msg("Motors stopped")
            elif action == 'reset':
                send_recv(ser, addr, CMD_RESET_ENCODERS)
                state.log_msg("Encoders reset")
            elif action == 'baud':
                stop_event.set()
                poll_thread.join(timeout=1.0)
                stop_event.clear()
                new_baud = action_set_baud(stdscr, ser, addr, state, cur_baud[0])
                cur_baud[0] = new_baud
                poll_thread = threading.Thread(
                    target=poller, args=(ser, addr, state, stop_event), daemon=True)
                poll_thread.start()
            elif action == 'pid':
                stop_event.set()
                poll_thread.join(timeout=1.0)
                stop_event.clear()
                action_set_pid(stdscr, ser, addr, state)
                poll_thread = threading.Thread(
                    target=poller, args=(ser, addr, state, stop_event), daemon=True)
                poll_thread.start()
            elif action == 'debug':
                show_debug[0] = not show_debug[0]
    finally:
        stop_event.set()
        poll_thread.join(timeout=1.0)


# ── Entry point ───────────────────────────────────────────────────────────────

addr = 0x80
port = "/dev/ttyO1"

def main():
    global addr, port
    parser = argparse.ArgumentParser(description="RoboClaw ncurses dashboard")
    parser.add_argument("--port", default="/dev/ttyO1")
    parser.add_argument("--baud", type=int, default=460800)
    parser.add_argument("--addr", type=lambda x: int(x, 0), default=0x80)
    args = parser.parse_args()

    addr = args.addr
    port = args.port

    try:
        ser = serial.Serial(args.port, args.baud, timeout=TIMEOUT)
        time.sleep(0.05)
        ser.reset_input_buffer()
    except serial.SerialException as e:
        print("ERROR: %s" % e)
        sys.exit(1)

    state = State()
    try:
        curses.wrapper(tui_main, ser, args.addr, state, args.baud)
    finally:
        ser.close()

if __name__ == "__main__":
    main()
