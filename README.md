# Rydful Beacon

A motion-triggered BLE beacon application for Nordic nRF52 microcontrollers using the Zephyr RTOS. The device remains in sleep mode until motion is detected via an LIS3DH accelerometer, then starts BLE advertising to enable presence detection.

Desinged for Rydful App: https://rydful.com

## Features

- **Low-Power Interrupt-Driven Wake** – CPU sleeps deeply and wakes only on accelerometer INT1 interrupt
- **Motion-Triggered Advertising** – BLE advertising starts only when motion is detected
- **Dual Motion Detection Algorithm**:
  - Sudden change detection for starts, stops, and turns
  - Variance-based detection for highway cruising (road vibrations)
- **Battery Monitoring** – Real-time battery voltage measurement with percentage calculation, broadcast via BLE manufacturer data
- **LED Feedback** – Visual indication of advertising state and startup sequence
- **Configurable Timeouts** – Advertising stops after configurable period of no motion (default: 60 seconds)
- **Debounce Filtering** – Requires multiple motion hits to confirm motion, reducing false positives

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

### Motion Detection Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `MOTION_THRESHOLD_MS2` | 0.25 m/s² | Threshold for sudden acceleration changes |
| `VARIANCE_THRESHOLD` | 0.01 | Variance threshold for vibration detection |
| `MIN_MOTION_HITS` | 3 | Required consecutive hits to confirm motion |
| `NO_MOTION_TIMEOUT_SEC` | 60 | Seconds of no motion before stopping advertising |
| `ACCEL_SAMPLE_INTERVAL_MS` | 100 | Accelerometer polling interval (when advertising) |

### Battery Monitoring

| Parameter | Default | Description |
|-----------|---------|-------------|
| `BATTERY_LOW_THRESHOLD_MV` | 2200 mV | 0% battery threshold |
| `BATTERY_FULL_MV` | 3000 mV | 100% battery threshold |
| `BATTERY_UPDATE_INTERVAL_SEC` | 30 | Battery status update interval |

### BLE Configuration

- **Device Name**: `Rydful_Beacon` (configurable via `CONFIG_BT_DEVICE_NAME`)
- **Service UUID**: `0xFEAA` (16-bit)
- **Manufacturer ID**: `0xFFFF` (development/testing)
- **Advertising Interval**: 1000-1500 ms (slow interval for power savings)
- **Mode**: Connectable, scannable

### BLE Advertising Data

The manufacturer-specific data contains:
- Bytes 0-1: Manufacturer ID
- Byte 2: Battery percentage (0-100)
- Byte 3: Low battery flag (0x00 or 0x01)

## Building

### Prerequisites

- [Zephyr SDK](https://docs.zephyrproject.org/latest/develop/getting_started/index.html) installed
- West tool configured
- nRF Connect SDK (optional, for Nordic-specific features)

### Build Commands

```bash
# Set up Zephyr environment
source ~/zephyrproject/zephyr/zephyr-env.sh

# Build for nRF52 DK (development/testing with external accelerometer)
west build -b nrf52dk_nrf52832 -p

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

## Usage

### nRF52 DK (Development)

1. **Power on** – LED blinks 3 times during startup
2. **Sleep mode** – LED off, no BLE advertising
3. **Motion detected** – LED turns on, BLE advertising starts
4. **No motion** – After 60 seconds, advertising stops

### Custom PCB (Production)

1. **Power on** – Device initializes (no LED feedback)
2. **Sleep mode** – Low power, no BLE advertising
3. **Motion detected** – BLE advertising starts
4. **No motion** – After 60 seconds, advertising stops and device returns to sleep

### Monitoring

Connect via serial console (115200 baud) to view logs:

```
==========================================
  Motion-Triggered BLE Beacon
==========================================
Sudden change threshold^2: 0.0625
Variance threshold: 0.0100
Min motion hits: 3 (debounce)
Sample interval: 100 ms
No-motion timeout: 60 seconds
Battery update interval: 30 seconds
Battery range: 2200 mV (0%) - 3000 mV (100%)
Device name: Rydful_Beacon
==========================================
```

## Enabled Zephyr Subsystems

- `CONFIG_GPIO` – LED control
- `CONFIG_LOG` – Logging (level 3)
- `CONFIG_BT` – Bluetooth stack
- `CONFIG_BT_PERIPHERAL` – Peripheral role
- `CONFIG_I2C` – Accelerometer communication
- `CONFIG_SENSOR` – Sensor subsystem
- `CONFIG_LIS2DH` – LIS3DH/LIS2DH driver with interrupt trigger support
- `CONFIG_ADC` – Battery voltage measurement
- `CONFIG_FPU` – Floating point support
- `CONFIG_NEWLIB_LIBC` – Math library support
- `CONFIG_PM_DEVICE` – Device power management
- `CONFIG_PM_DEVICE_RUNTIME` – Runtime power management

## License

SPDX-License-Identifier: Apache-2.0
