#!/usr/bin/env python3
"""
roboclaw_tui.py — RoboClaw 2x7A ncurses dashboard
Part of roboclaw-util · github.com/ryanl/roboclaw-util

Usage:
  python3 roboclaw_tui.py [--port /dev/ttyO1] [--baud 460800] [--addr 0x80]

Keys:
  s     — stop both motors (zero speed)
  e     — toggle e-stop pin state (if wired)
  r     — reset encoders
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

PORT    = "/dev/ttyO1"
BAUD    = 460800
ADDRESS = 0x80
TIMEOUT = 0.1
POLL_HZ = 10   # telemetry poll rate

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
CMD_DRIVE_M1_SIGNED     = 32
CMD_DRIVE_M2_SIGNED     = 33
CMD_DRIVE_M1M2_SPEED    = 37
CMD_READ_MOTOR_CURRENTS = 49
CMD_READ_M1_VELPID      = 55
CMD_READ_M2_VELPID      = 56
CMD_READ_TEMP           = 82
CMD_READ_STATUS         = 90
CMD_WRITE_NVM           = 94

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

# ── Telemetry state ───────────────────────────────────────────────────────────

class State:
    def __init__(self):
        self.lock = threading.Lock()
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
        self.m1_qpps = 0
        self.m2_kp = self.m2_ki = self.m2_kd = 0.0
        self.m2_qpps = 0
        self.poll_count  = 0
        self.errors      = 0
        self.log         = collections.deque(maxlen=6)
        self.debug_lines = collections.deque(maxlen=4)
        self.start_time  = time.time()

    def log_msg(self, msg):
        ts = time.strftime("%H:%M:%S")
        with self.lock:
            self.log.append("[%s] %s" % (ts, msg))

# ── Poller thread ─────────────────────────────────────────────────────────────

def poller(ser, addr, state, stop_event):
    # Read firmware once
    ser.reset_input_buffer()
    send_cmd(ser, addr, CMD_READ_VERSION)
    time.sleep(0.15)
    raw = ser.read(48)
    if raw:
        fw = raw[:-2].decode("ascii", errors="replace").strip().strip("\x00")
        with state.lock:
            state.firmware = fw
            state.connected = True
        state.log_msg("Connected: %s" % fw)
    else:
        state.log_msg("WARNING: no firmware response")

    # Read PID params once (static)
    for ch in (1, 2):
        cmd = CMD_READ_M1_VELPID if ch == 1 else CMD_READ_M2_VELPID
        d = send_recv(ser, addr, cmd, recv_n=16)
        if d:
            p, i, dv, q = struct.unpack(">IIII", d)
            with state.lock:
                if ch == 1:
                    state.m1_kp = p / 65536.0
                    state.m1_ki = i / 65536.0
                    state.m1_kd = dv / 65536.0
                    state.m1_qpps = q
                else:
                    state.m2_kp = p / 65536.0
                    state.m2_ki = i / 65536.0
                    state.m2_kd = dv / 65536.0
                    state.m2_qpps = q

    interval = 1.0 / POLL_HZ
    while not stop_event.is_set():
        t0 = time.time()
        try:
            d = send_recv(ser, addr, CMD_READ_MAIN_BATT, recv_n=2)
            if d:
                with state.lock:
                    state.vbat = struct.unpack(">H", d)[0] / 10.0

            d = send_recv(ser, addr, CMD_READ_LOGIC_BATT, recv_n=2)
            if d:
                with state.lock:
                    state.vlogic = struct.unpack(">H", d)[0] / 10.0

            d = send_recv(ser, addr, CMD_READ_TEMP, recv_n=2)
            if d:
                with state.lock:
                    state.temp = struct.unpack(">H", d)[0] / 10.0

            d = send_recv(ser, addr, CMD_READ_STATUS, recv_n=4)
            if d:
                with state.lock:
                    state.status_flags = struct.unpack(">I", d)[0]

            d = send_recv(ser, addr, CMD_READ_MOTOR_CURRENTS, recv_n=4)
            if d:
                with state.lock:
                    state.m1_current = struct.unpack(">H", d[0:2])[0] / 100.0
                    state.m2_current = struct.unpack(">H", d[2:4])[0] / 100.0

            d = send_recv(ser, addr, CMD_READ_SPEED1, recv_n=5)
            if d:
                with state.lock:
                    state.m1_speed = struct.unpack(">i", d[:4])[0]

            d = send_recv(ser, addr, CMD_READ_SPEED2, recv_n=5)
            if d:
                with state.lock:
                    state.m2_speed = struct.unpack(">i", d[:4])[0]

            d = send_recv(ser, addr, CMD_READ_ENC1, recv_n=5)
            if d:
                with state.lock:
                    state.m1_enc = struct.unpack(">i", d[:4])[0]

            d = send_recv(ser, addr, CMD_READ_ENC2, recv_n=5)
            if d:
                with state.lock:
                    state.m2_enc = struct.unpack(">i", d[:4])[0]

            with state.lock:
                state.poll_count += 1
                state.connected = True

        except Exception as ex:
            with state.lock:
                state.errors += 1
                state.connected = False
            state.log_msg("Poll error: %s" % str(ex))

        elapsed = time.time() - t0
        sleep_t = max(0, interval - elapsed)
        stop_event.wait(sleep_t)


# ── Drawing helpers ───────────────────────────────────────────────────────────

def bar(val, maxval, width, full_char="█", empty_char="░"):
    if maxval <= 0:
        maxval = 1
    filled = int(round(min(val, maxval) / maxval * width))
    filled = max(0, min(filled, width))
    return full_char * filled + empty_char * (width - filled)

def clamp_str(s, width):
    if len(s) > width:
        return s[:width]
    return s

def safe_addstr(win, y, x, s, attr=0):
    h, w = win.getmaxyx()
    if y < 0 or y >= h or x < 0 or x >= w:
        return
    max_len = w - x - 1
    if max_len <= 0:
        return
    try:
        win.addstr(y, x, s[:max_len], attr)
    except curses.error:
        pass

def hline(win, y, x, width, attr=0):
    safe_addstr(win, y, x, "─" * width, attr)

def box_title(win, y, x, width, title, attr=0, title_attr=0):
    safe_addstr(win, y, x, "┌", attr)
    safe_addstr(win, y, x+1, "─" * (width-2), attr)
    safe_addstr(win, y, x+width-1, "┐", attr)
    tx = x + (width - len(title) - 2) // 2
    safe_addstr(win, y, tx, " %s " % title, title_attr if title_attr else attr)

def box_bottom(win, y, x, width, attr=0):
    safe_addstr(win, y, x, "└", attr)
    safe_addstr(win, y, x+1, "─" * (width-2), attr)
    safe_addstr(win, y, x+width-1, "┘", attr)

def box_side(win, y, x, width, attr=0):
    safe_addstr(win, y, x, "│", attr)
    safe_addstr(win, y, x+width-1, "│", attr)


# ── Main TUI ─────────────────────────────────────────────────────────────────

def draw(stdscr, state, show_debug):
    curses.curs_set(0)
    stdscr.nodelay(True)

    # Color pairs
    curses.init_pair(1,  curses.COLOR_CYAN,    curses.COLOR_BLACK)  # header/border
    curses.init_pair(2,  curses.COLOR_GREEN,   curses.COLOR_BLACK)  # ok / good values
    curses.init_pair(3,  curses.COLOR_YELLOW,  curses.COLOR_BLACK)  # warning
    curses.init_pair(4,  curses.COLOR_RED,     curses.COLOR_BLACK)  # error / fault
    curses.init_pair(5,  curses.COLOR_WHITE,   curses.COLOR_BLACK)  # normal text
    curses.init_pair(6,  curses.COLOR_BLACK,   curses.COLOR_CYAN)   # highlight bar
    curses.init_pair(7,  curses.COLOR_MAGENTA, curses.COLOR_BLACK)  # accent
    curses.init_pair(8,  curses.COLOR_WHITE,   curses.COLOR_BLACK)  # dim

    C_BORDER  = curses.color_pair(1)
    C_OK      = curses.color_pair(2)
    C_WARN    = curses.color_pair(3)
    C_ERR     = curses.color_pair(4)
    C_NORMAL  = curses.color_pair(5)
    C_HILIGHT = curses.color_pair(6)
    C_ACCENT  = curses.color_pair(7)
    C_DIM     = curses.color_pair(8) | curses.A_DIM
    C_BOLD    = curses.color_pair(5) | curses.A_BOLD
    C_HEAD    = curses.color_pair(1) | curses.A_BOLD

    LOGO = "RoboClaw Monitor"
    SUBLOGO = "2x7A · Packet Serial"

    last_draw = 0.0

    while True:
        key = stdscr.getch()
        if key == ord('q'):
            return 'quit'
        elif key == ord('s'):
            return 'stop'
        elif key == ord('r'):
            return 'reset_enc'
        elif key == ord('e'):
            return 'estop'
        elif key == ord('d'):
            return 'debug'

        now = time.time()
        if now - last_draw < 0.08:   # ~12 fps
            time.sleep(0.02)
            continue
        last_draw = now

        h, w = stdscr.getmaxyx()
        if h < 24 or w < 80:
            stdscr.clear()
            safe_addstr(stdscr, 0, 0, "Terminal too small (need 80x24+)", C_ERR)
            stdscr.refresh()
            continue

        stdscr.erase()

        with state.lock:
            s = state.__dict__.copy()
            log_lines = list(state.log)
            debug_lines = list(state.debug_lines)

        uptime = int(now - s["start_time"])
        up_str = "%02d:%02d:%02d" % (uptime//3600, (uptime%3600)//60, uptime%60)

        # ── Header bar ────────────────────────────────────────────────────────
        safe_addstr(stdscr, 0, 0, " " * w, C_HILIGHT)
        safe_addstr(stdscr, 0, 2, LOGO, C_HILIGHT | curses.A_BOLD)
        safe_addstr(stdscr, 0, 2 + len(LOGO) + 2, SUBLOGO, C_HILIGHT)
        conn_str = "● CONNECTED" if s["connected"] else "○ OFFLINE"
        conn_attr = C_HILIGHT | curses.A_BOLD
        safe_addstr(stdscr, 0, w - len(conn_str) - len(up_str) - 4, conn_str, conn_attr)
        safe_addstr(stdscr, 0, w - len(up_str) - 2, up_str, C_HILIGHT)

        # Layout columns
        left_w  = 22
        right_w = 22
        mid_w   = w - left_w - right_w - 2
        mid_x   = left_w + 1
        right_x = w - right_w

        row = 1   # current draw row

        # ── Left panel: SYSTEM ────────────────────────────────────────────────
        box_title(stdscr, row,   0, left_w, "SYSTEM", C_BORDER, C_HEAD)
        box_side( stdscr, row+1, 0, left_w, C_BORDER)
        box_side( stdscr, row+2, 0, left_w, C_BORDER)
        box_side( stdscr, row+3, 0, left_w, C_BORDER)
        box_side( stdscr, row+4, 0, left_w, C_BORDER)
        box_side( stdscr, row+5, 0, left_w, C_BORDER)
        box_side( stdscr, row+6, 0, left_w, C_BORDER)
        box_bottom(stdscr, row+7, 0, left_w, C_BORDER)

        # Voltage with bar
        vbat = s["vbat"]
        v_attr = C_OK if vbat > 10.5 else (C_WARN if vbat > 9.6 else C_ERR)
        safe_addstr(stdscr, row+1, 2, "VBAT", C_DIM)
        safe_addstr(stdscr, row+1, 7, "%5.2fV" % vbat, v_attr | curses.A_BOLD)
        vbar = bar(vbat - 9.0, 3.6, left_w - 4)
        safe_addstr(stdscr, row+2, 2, vbar[:left_w-4], v_attr)

        # Temp
        temp = s["temp"]
        t_attr = C_OK if temp < 60 else (C_WARN if temp < 85 else C_ERR)
        safe_addstr(stdscr, row+3, 2, "TEMP", C_DIM)
        safe_addstr(stdscr, row+3, 7, "%5.1f°C" % temp, t_attr | curses.A_BOLD)
        tbar = bar(temp, 100.0, left_w - 4)
        safe_addstr(stdscr, row+4, 2, tbar[:left_w-4], t_attr)

        # Logic batt
        safe_addstr(stdscr, row+5, 2, "LOGIC", C_DIM)
        safe_addstr(stdscr, row+5, 8, "%4.1fV" % s["vlogic"], C_NORMAL)

        # Poll stats
        safe_addstr(stdscr, row+6, 2, "POLL", C_DIM)
        safe_addstr(stdscr, row+6, 7, "%5d" % s["poll_count"], C_DIM)
        safe_addstr(stdscr, row+6, 13, "ERR", C_DIM)
        err_attr = C_ERR if s["errors"] > 0 else C_DIM
        safe_addstr(stdscr, row+6, 17, "%3d" % s["errors"], err_attr)

        # ── Left panel: STATUS FLAGS ──────────────────────────────────────────
        flag_row = row + 9
        box_title(stdscr, flag_row,   0, left_w, "STATUS", C_BORDER, C_HEAD)
        flags = s["status_flags"]
        active = [(bit, name) for bit, name in STATUS_NAMES.items() if flags & (1 << bit)]
        if not active:
            box_side(stdscr, flag_row+1, 0, left_w, C_BORDER)
            safe_addstr(stdscr, flag_row+1, 2, "ALL CLEAR", C_OK | curses.A_BOLD)
            for i in range(2, 5):
                box_side(stdscr, flag_row+i, 0, left_w, C_BORDER)
            box_bottom(stdscr, flag_row+5, 0, left_w, C_BORDER)
        else:
            for i in range(4):
                box_side(stdscr, flag_row+1+i, 0, left_w, C_BORDER)
                if i < len(active):
                    bit, name = active[i]
                    fa = C_ERR if bit in (3,4,6,8) else C_WARN
                    safe_addstr(stdscr, flag_row+1+i, 2, "▲ %-16s" % name[:16], fa | curses.A_BOLD)
            box_bottom(stdscr, flag_row+5, 0, left_w, C_BORDER)

        # ── Right panel: CONTROLS ─────────────────────────────────────────────
        box_title(stdscr, row, right_x, right_w, "CONTROLS", C_BORDER, C_HEAD)
        ctrl_lines = [
            ("s", "STOP MOTORS"),
            ("e", "TOGGLE E-STOP"),
            ("r", "RESET ENCODERS"),
            ("d", "DEBUG OVERLAY"),
            ("q", "QUIT"),
        ]
        for i, (key_ch, label) in enumerate(ctrl_lines):
            box_side(stdscr, row+1+i, right_x, right_w, C_BORDER)
            safe_addstr(stdscr, row+1+i, right_x+2, "[", C_DIM)
            safe_addstr(stdscr, row+1+i, right_x+3, key_ch, C_ACCENT | curses.A_BOLD)
            safe_addstr(stdscr, row+1+i, right_x+4, "]", C_DIM)
            safe_addstr(stdscr, row+1+i, right_x+6, label, C_NORMAL)
        for i in range(len(ctrl_lines), 6):
            box_side(stdscr, row+1+i, right_x, right_w, C_BORDER)
        box_bottom(stdscr, row+7, right_x, right_w, C_BORDER)

        # Firmware in right panel
        fw_row = row + 9
        box_title(stdscr, fw_row, right_x, right_w, "FIRMWARE", C_BORDER, C_HEAD)
        fw = s["firmware"]
        box_side(stdscr, fw_row+1, right_x, right_w, C_BORDER)
        safe_addstr(stdscr, fw_row+1, right_x+2, fw[:right_w-4], C_OK)
        box_side(stdscr, fw_row+2, right_x, right_w, C_BORDER)
        addr_str = "ADDR 0x%02X  %dbd" % (ADDRESS, BAUD)
        safe_addstr(stdscr, fw_row+2, right_x+2, addr_str[:right_w-4], C_DIM)
        box_side(stdscr, fw_row+3, right_x, right_w, C_BORDER)
        port_str = PORT
        safe_addstr(stdscr, fw_row+3, right_x+2, port_str[:right_w-4], C_DIM)
        for i in range(4, 6):
            box_side(stdscr, fw_row+i, right_x, right_w, C_BORDER)
        box_bottom(stdscr, fw_row+5, right_x, right_w, C_BORDER)

        # ── Center: Motor panels ──────────────────────────────────────────────
        motor_h = 8
        for m_idx, (label, speed, current, enc, kp, ki, kd, qpps) in enumerate([
            ("MOTOR 1", s["m1_speed"], s["m1_current"], s["m1_enc"],
             s["m1_kp"], s["m1_ki"], s["m1_kd"], s["m1_qpps"]),
            ("MOTOR 2", s["m2_speed"], s["m2_current"], s["m2_enc"],
             s["m2_kp"], s["m2_ki"], s["m2_kd"], s["m2_qpps"]),
        ]):
            mr = row + m_idx * (motor_h + 1)
            box_title(stdscr, mr, mid_x, mid_w, label, C_BORDER, C_HEAD)
            for i in range(1, motor_h):
                box_side(stdscr, mr+i, mid_x, mid_w, C_BORDER)
            box_bottom(stdscr, mr+motor_h, mid_x, mid_w, C_BORDER)

            inner_w = mid_w - 4
            cx = mid_x + 2

            # Speed
            spd_pct = min(abs(speed) / max(qpps, 1), 1.0) if qpps > 0 else 0
            spd_attr = C_OK if abs(speed) < qpps * 0.9 else C_WARN
            spd_dir = "►" if speed > 0 else ("◄" if speed < 0 else "■")
            safe_addstr(stdscr, mr+1, cx, "SPD", C_DIM)
            safe_addstr(stdscr, mr+1, cx+4, "%s %6d pps" % (spd_dir, speed), spd_attr | curses.A_BOLD)
            spd_bar_w = inner_w - 2
            spd_bar = bar(abs(speed), max(qpps, 1), spd_bar_w)
            safe_addstr(stdscr, mr+2, cx, spd_bar[:spd_bar_w], spd_attr)

            # Current
            c_attr = C_OK if current < 4.0 else (C_WARN if current < 6.5 else C_ERR)
            safe_addstr(stdscr, mr+3, cx, "AMP", C_DIM)
            safe_addstr(stdscr, mr+3, cx+4, "%5.2f A" % current, c_attr | curses.A_BOLD)
            amp_bar = bar(current, 7.5, inner_w - 2)
            safe_addstr(stdscr, mr+4, cx, amp_bar[:inner_w-2], c_attr)

            # Encoder + PID params inline
            safe_addstr(stdscr, mr+5, cx, "ENC", C_DIM)
            safe_addstr(stdscr, mr+5, cx+4, "%12d" % enc, C_NORMAL | curses.A_BOLD)

            pid_str = "P%-5.2f I%-5.2f D%-5.2f  QPPS %d" % (kp, ki, kd, qpps)
            safe_addstr(stdscr, mr+6, cx, pid_str[:inner_w], C_DIM)

        # ── Log strip ─────────────────────────────────────────────────────────
        log_start = h - len(log_lines) - 2
        box_title(stdscr, log_start - 1, 0, w, "LOG", C_BORDER, C_HEAD)
        for i, line in enumerate(log_lines):
            age = len(log_lines) - i
            attr = C_NORMAL if age <= 2 else C_DIM
            safe_addstr(stdscr, log_start + i, 2, clamp_str(line, w - 4), attr)

        # ── Debug overlay ─────────────────────────────────────────────────────
        if show_debug[0]:
            drow = 2
            safe_addstr(stdscr, drow, mid_x, " DEBUG ", C_ACCENT | curses.A_BOLD)
            for i, dl in enumerate(debug_lines):
                safe_addstr(stdscr, drow+1+i, mid_x, dl[:mid_w-2], C_ACCENT)

        stdscr.refresh()


def tui_main(stdscr, ser, addr, state):
    show_debug = [False]
    stop_event  = threading.Event()

    poll_thread = threading.Thread(
        target=poller, args=(ser, addr, state, stop_event), daemon=True)
    poll_thread.start()

    try:
        while True:
            action = draw(stdscr, state, show_debug)
            if action == 'quit':
                break
            elif action == 'stop':
                send_recv(ser, addr, CMD_DRIVE_M1M2_SPEED,
                          payload=struct.pack(">ii", 0, 0))
                state.log_msg("Motors stopped")
            elif action == 'reset_enc':
                send_recv(ser, addr, CMD_RESET_ENCODERS)
                state.log_msg("Encoders reset")
            elif action == 'estop':
                state.log_msg("E-stop toggle (wire BBB GPIO to S4)")
            elif action == 'debug':
                show_debug[0] = not show_debug[0]
    finally:
        stop_event.set()
        poll_thread.join(timeout=1.0)


def main():
    parser = argparse.ArgumentParser(description="RoboClaw ncurses dashboard")
    parser.add_argument("--port", default=PORT)
    parser.add_argument("--baud", type=int, default=BAUD)
    parser.add_argument("--addr", type=lambda x: int(x, 0), default=ADDRESS)
    args = parser.parse_args()

    global ADDRESS, PORT, BAUD
    ADDRESS = args.addr
    PORT    = args.port
    BAUD    = args.baud

    try:
        ser = serial.Serial(args.port, args.baud, timeout=TIMEOUT)
        time.sleep(0.05)
        ser.reset_input_buffer()
    except serial.SerialException as e:
        print("ERROR: %s" % e)
        sys.exit(1)

    state = State()
    try:
        curses.wrapper(tui_main, ser, args.addr, state)
    finally:
        ser.close()

if __name__ == "__main__":
    main()
