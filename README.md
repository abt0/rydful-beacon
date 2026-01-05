# Rydful Beacon

A motion-triggered BLE beacon application for Nordic nRF52 microcontrollers using the Zephyr RTOS. The device remains in sleep mode until motion is detected via an LIS3DH accelerometer, then starts BLE advertising to enable presence detection.

Designed for Rydful App: https://rydful.com

## Features

- **Hardware-Driven Motion Detection** – LIS3DH accelerometer handles motion detection internally, MCU only wakes on threshold events
- **Ultra-Low Power Sleep** – CPU sleeps indefinitely until real motion occurs (~13 µA total idle current)
- **Motion-Triggered Advertising** – BLE advertising starts only when motion exceeds hardware threshold
- **Battery Monitoring** – Real-time battery voltage measurement with percentage calculation, broadcast via BLE manufacturer data
- **LED Feedback** – Visual indication of advertising state and startup sequence
- **Configurable Timeouts** – Advertising stops after configurable period of no motion (default: 60 seconds)
- **High-Pass Filtered Detection** – Gravity is filtered out, only acceleration changes trigger wake

## Hardware Requirements

- **MCU**: Nordic nRF52832 (nRF52 DK)
- **Accelerometer**: LIS3DH or LIS2DH (I2C interface)
- **LED**: Connected to `led0` alias (built-in on dev kits)

### Wiring (nRF52 DK with LIS3DH)

| LIS3DH Pin | nRF52 DK Pin | Description |
|------------|--------------|-------------|
| SDA        | P0.26        | I2C Data    |
| SCL        | P0.27        | I2C Clock   |
| CS         | VDD          | I2C mode select |
| INT1       | P0.02        | Motion interrupt (required for low-power wake) |

## Project Structure

```
rydful-beacon/
├── CMakeLists.txt              # Build configuration
├── Kconfig                     # Custom Kconfig options
├── prj.conf                    # Zephyr project configuration
├── hardware/                   # Hardware description
├── src/
│   └── main.c                  # Main application source
└── boards/
    ├── nrf52dk_nrf52832.overlay    # nRF52 DK device tree overlay
    └── arm/
        └── rydful_custom/          # Custom PCB board definition
            ├── rydful_custom.dts           # Device tree
            ├── rydful_custom-pinctrl.dtsi  # Pin control
            ├── rydful_custom_defconfig     # Default Kconfig
            ├── rydful_custom.yaml          # Board metadata
            ├── Kconfig.board               # Board Kconfig
            ├── Kconfig.defconfig           # Kconfig defaults
            └── board.cmake                 # Flash/debug config
```

## Supported Boards

| Board | Description | Use Case |
|-------|-------------|----------|
| `nrf52dk_nrf52832` | Nordic nRF52 DK | Development/testing with external LIS3DH |
| `rydful_custom` | Custom PCB | Production board with integrated LIS3DHTR |

## Configuration

### Hardware Motion Detection Parameters

These are defined in `src/main.c`:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `HW_MOTION_THRESHOLD_MG` | 48 mg | Acceleration threshold for motion detection |
| `HW_MOTION_DURATION` | 1 sample | Required samples above threshold (debounce) |
| `NO_MOTION_TIMEOUT_SEC` | 60 | Seconds of no motion before stopping advertising |

The LIS3DH uses its internal high-pass filter to remove gravity, so only acceleration *changes* trigger the interrupt.

### Battery Monitoring

| Parameter | Default | Description |
|-----------|---------|-------------|
| `BATTERY_LOW_THRESHOLD_MV` | 2000 mV | 0% battery threshold |
| `BATTERY_FULL_MV` | 3000 mV | 100% battery threshold |
| `BATTERY_UPDATE_INTERVAL_SEC` | 30 | Battery status update interval |

### BLE Configuration

- **Device Name**: `Rydful_Beacon` (configurable via `CONFIG_BT_DEVICE_NAME`)
- **Service UUID**: `0xFEAA` (16-bit)
- **Manufacturer ID**: `0xFFFF` (development/testing)
- **Advertising Interval**: 250-400 ms
- **Mode**: Connectable, scannable

### BLE Advertising Data

The manufacturer-specific data contains:
- Bytes 0-1: Manufacturer ID
- Byte 2: Battery percentage (0-100)
- Byte 3: Low battery flag (0x00 or 0x01)

## Building

### Prerequisites

- [nRF Connect SDK](https://docs.nordicsemi.com/bundle/ncs-latest/page/nrf/installation.html) installed
- West tool configured

### Build Commands

```bash
# Build for nRF52 DK (development/testing with external accelerometer)
west build -b nrf52dk/nrf52832 -p

# Build for Rydful Custom PCB (production board)
west build -b rydful_custom -p -- -DBOARD_ROOT=.
```

### Flash

```bash
# Flash nRF52 DK (direct connection)
west flash
```

## Flashing Custom PCB via nRF52 DK

The nRF52 DK can be used as a J-Link programmer to flash the custom PCB.

### Hardware Setup

1. **Disconnect** the nRF52 DK from USB
2. **Set the nRF power switch** to "VDD" (not "USB") on the DK
3. **Connect SWD wires** from DK P20 header to your custom PCB:

| DK P20 Pin | Custom PCB | Description |
|------------|------------|-------------|
| SWDIO      | SWDIO      | Debug data  |
| SWDCLK     | SWDCLK     | Debug clock |
| GND        | GND        | Ground      |
| VTG        | VDD        | Target voltage sense (optional) |

4. **Power the custom PCB** via its own power source (battery or external supply)
5. **Connect DK to USB** for programming

### Flash Command

```bash
# Build for custom PCB
west build -b rydful_custom -p -- -DBOARD_ROOT=.

# Flash via J-Link
west flash
```

### Troubleshooting

- **"No target detected"** – Check SWD wiring, ensure custom PCB is powered
- **"Could not connect"** – Verify SWDIO/SWDCLK connections, check for shorts
- **Use nRF Connect Programmer** – Alternative GUI tool for flashing `.hex` files from `build/zephyr/zephyr.hex`

### Using nrfjprog (Alternative)

```bash
# Erase and flash
nrfjprog --program build/zephyr/zephyr.hex --chiperase --verify
nrfjprog --reset
```

## How It Works

### State Machine

```
┌─────────────────────────────────────────────────────────────┐
│  SLEEP MODE                                                 │
│  • LIS3DH: Hardware threshold interrupt (>48mg)             │
│  • nRF52: System ON sleep (~1.9 µA)                         │
│  • Total: ~13 µA                                            │
└──────────────────────┬──────────────────────────────────────┘
                       │ Motion exceeds threshold
                       ▼
┌─────────────────────────────────────────────────────────────┐
│  ACTIVE MODE                                                │
│  • BLE advertising active                                   │
│  • Each motion interrupt resets 60-second timeout           │
│  • LED ON                                                   │
└──────────────────────┬──────────────────────────────────────┘
                       │ No motion for 60 seconds
                       ▼
              Back to SLEEP MODE
```

### Usage

1. **Power on** – LED blinks 3 times during startup
2. **Sleep mode** – LED off, no BLE advertising, ultra-low power
3. **Motion detected** – LED turns on, BLE advertising starts
4. **Continuous motion** – Each motion event resets the 60-second timeout
5. **No motion** – After 60 seconds of no motion, advertising stops

### Monitoring

Connect via serial console (115200 baud) to view logs:

```
==========================================
  Motion-Triggered BLE Beacon
==========================================
Mode: Hardware motion detection
Motion threshold: 48 mg
Motion duration: 1 samples
No-motion timeout: 60 seconds
Battery update interval: 30 seconds
Battery range: 2000 mV (0%) - 3000 mV (100%)
Device name: Rydful_Beacon
==========================================
Hardware motion mode active (threshold: 48mg, duration: 1 samples)
(Move the device to start BLE advertising)
Entering sleep (waiting for motion)...
Motion detected - waking up!
>>> BLE advertising STARTED (motion detected)
```

## Power Consumption

| State | Current | Description |
|-------|---------|-------------|
| Sleep | ~13 µA | nRF52 System ON + LIS3DH @ 10Hz |
| Active (advertising) | ~1-3 mA avg | BLE advertising + occasional motion interrupts |

### Battery Life Estimate (2x AA batteries @ 2500 mAh)

| Usage Pattern | Estimated Life |
|---------------|----------------|
| Mostly idle (parked car) | ~1.5-2 years |
| Active 8 hours/day | ~6-12 months |

## Enabled Zephyr Subsystems

- `CONFIG_GPIO` – LED control
- `CONFIG_LOG` – Logging
- `CONFIG_BT` – Bluetooth stack
- `CONFIG_BT_PERIPHERAL` – Peripheral role
- `CONFIG_I2C` – Accelerometer communication
- `CONFIG_SENSOR` – Sensor subsystem
- `CONFIG_LIS2DH` – LIS3DH/LIS2DH driver (trigger disabled, using direct register access)
- `CONFIG_ADC` – Battery voltage measurement
- `CONFIG_FPU` – Floating point support (for diagnostics)
- `CONFIG_NEWLIB_LIBC` – Math library support

## Optional Configuration

Enable hardware diagnostics on startup (verifies accelerometer is working):

```conf
CONFIG_BEACON_DIAGNOSTICS=y
```

## License

SPDX-License-Identifier: Apache-2.0
