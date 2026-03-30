# roboclaw_tools

Python utility for configuring and monitoring a RoboClaw 2x7A motor controller
over packet serial — no USB required.

Built for the **balance_bot** project (BeagleBone Blue + RoboClaw 2x7A).

---

## Hardware

| Signal | BBB pin | RoboClaw pin |
|--------|---------|--------------|
| TX     | UART1 TX (`/dev/ttyO1`) | S1 (RX) |
| RX     | UART1 RX (`/dev/ttyO1`) | S2 (TX) |
| GND    | GND     | GND          |
| E-Stop | GPIO    | S4 (I/O)     |

- Baud: **460800**, Address: **0x80**
- S4 is active-low with internal pull-up. Wire a BBB GPIO to S4 I/O pin.
  Add a 10k pull-down on the S4 side so it fails safe if the BBB is unpowered.
- Use `estop_config --no-latch` so the robot re-enables when the BBB comes back.

---

## Requirements

```bash
pip install pyserial
```

Python 3.6+ (no f-strings, no `|` type unions — runs on BBB Debian).

---

## Usage

```bash
python3 roboclaw.py <command> [args] [options]
```

### Options

| Flag | Default | Description |
|------|---------|-------------|
| `--port PORT` | `/dev/ttyO1` | Serial port |
| `--baud BAUD` | `460800` | Baud rate |
| `--addr ADDR` | `0x80` | RoboClaw address (hex) |
| `--no-latch` | — | Non-latching e-stop |
| `--debug` | — | Print raw TX/RX bytes |

---

## Commands

### Diagnostics

```bash
# Test comms — dumps raw bytes
python3 roboclaw.py probe

# Full status: firmware, voltage, temp, error flags
python3 roboclaw.py status

# Encoder counts and speeds
python3 roboclaw.py encoders

# Motor currents
python3 roboclaw.py currents

# Live monitoring table (default 1s interval)
python3 roboclaw.py monitor
python3 roboclaw.py monitor 0.5
```

### PID Configuration

```bash
# Read current velocity PID
python3 roboclaw.py pid1
python3 roboclaw.py pid2

# Set velocity PID and save to NVM
python3 roboclaw.py setpid1 1.0 0.5 0.25 2500
python3 roboclaw.py setpid2 1.0 0.5 0.25 2500
#                           Kp  Ki  Kd   QPPS

# Measure real-world QPPS (spins motors at 50% duty)
python3 roboclaw.py measure_qpps
```

> **Note:** QPPS is the encoder pulses-per-second at full throttle for your
> motors. Measure it with `measure_qpps` and use ~90% of that value.
> balance_bot uses RS-555 motors, 5.2:1 gearbox, 145.1 PPR → ~2500 QPPS.

### E-Stop Configuration

```bash
# Configure S3/S4/S5 as latching e-stop inputs (saves to NVM)
python3 roboclaw.py estop_config

# Non-latching (recommended for BBB GPIO watchdog — re-enables on BBB restart)
python3 roboclaw.py estop_config --no-latch
```

### Config / Save

```bash
# Read and decode config register
python3 roboclaw.py read_config

# Emergency motor stop (does NOT save)
python3 roboclaw.py stop

# Save all settings to NVM (unit resets after)
python3 roboclaw.py save
```

---

## Save / NVM notes

`save`, `setpid1/2`, and `estop_config` all call `WriteNVM` with the unlock
key `0xE22EAB7A`. The unit resets ~1s after the ACK — the script waits 3s
automatically before returning. This is the correct sequence for firmware 4.x.
Earlier scripts using plain cmd 94 without the key did not persist settings.

---

## E-Stop wiring

```
BBB GPIO ──────────────── S4 (I/O pin, board edge)
                │
               10k
                │
               GND ─────── S4 GND pin (inside pin)
```

In firmware: `estop_config --no-latch`

In balance_bot C firmware, hold GPIO high while process is running. On
process exit / BBB shutdown, GPIO goes low → S4 pulled low → motors stop.

```c
// init
rc_gpio_init(GPIO_CHIP, ESTOP_PIN, RCGPIO_OUTPUT);
rc_gpio_set_value(GPIO_CHIP, ESTOP_PIN, 1);   // enable

// on shutdown
rc_gpio_set_value(GPIO_CHIP, ESTOP_PIN, 0);   // e-stop
```

---

## monitor output

```
  Time    Vbat    Temp     M1A     M2A          Enc1          Enc2      Spd1      Spd2
   0.0    11.1    29.3    0.00    0.00             0             0         0         0
   1.0    11.1    29.3    1.23    1.19          1450          1448      2487      2491
```
