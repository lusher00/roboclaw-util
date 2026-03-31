#!/usr/bin/env python3
"""
roboclaw_reset.py — drive e-stop GPIO high then reset RoboClaw via WriteNVM.
No pyserial dependency — uses termios + sysfs directly.
Always exits 0 so a failure never blocks the service from starting.
"""
import os, sys, struct, time, termios

PORT       = '/dev/ttyO1'
BAUD       = 460800
ADDR       = 0x80
ESTOP_GPIO = 57   # GPIO1_25

BAUD_MAP = {
    460800: termios.B460800,
    115200: termios.B115200,
     38400: termios.B38400,
      9600: termios.B9600,
}

def crc16(data):
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = (crc << 1) ^ 0x1021 if crc & 0x8000 else crc << 1
    return crc & 0xFFFF

def gpio_high(gpio):
    try:
        gpio_path = "/sys/class/gpio/gpio%d" % gpio
        if not os.path.exists(gpio_path):
            with open("/sys/class/gpio/export", "w") as f:
                f.write(str(gpio))
            time.sleep(0.05)
        with open("%s/direction" % gpio_path, "w") as f:
            f.write("out")
        with open("%s/value" % gpio_path, "w") as f:
            f.write("1")
        print("roboclaw_reset: GPIO%d -> HIGH (e-stop deasserted)" % gpio)
    except Exception as e:
        print("roboclaw_reset: GPIO%d warning — %s" % (gpio, e))

def open_port(path, baud):
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    os.set_blocking(fd, True)
    attrs = termios.tcgetattr(fd)
    attrs[4] = BAUD_MAP[baud]
    attrs[5] = BAUD_MAP[baud]
    attrs[0] = 0
    attrs[1] = 0
    attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
    attrs[3] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    termios.tcflush(fd, termios.TCIOFLUSH)
    return fd

def send_acked(fd, payload, timeout=1.0):
    crc = crc16(payload)
    os.write(fd, payload + struct.pack('>H', crc))
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            b = os.read(fd, 1)
            if b and b[0] == 0xFF:
                return True
        except BlockingIOError:
            time.sleep(0.01)
    return False

try:
    # Step 1: drive e-stop high FIRST so RoboClaw sees it on reset
    gpio_high(ESTOP_GPIO)
    time.sleep(0.1)

    # Step 2: WriteNVM reset
    fd = open_port(PORT, BAUD)
    time.sleep(0.1)
    payload = struct.pack('>BBI', ADDR, 94, 0xE22EAB7A)
    ok = send_acked(fd, payload, timeout=1.0)
    os.close(fd)
    if ok:
        print("roboclaw_reset: WriteNVM OK — waiting 3s for unit to come up")
        time.sleep(3.0)
    else:
        print("roboclaw_reset: no ACK — unit may already be reset, continuing")
        time.sleep(1.0)

except Exception as e:
    print("roboclaw_reset: error — %s, continuing anyway" % e)

sys.exit(0)