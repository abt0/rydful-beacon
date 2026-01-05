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

### Current Draw by State

| State | Current | Description |
|-------|---------|-------------|
| Sleep | ~13 µA | nRF52 System ON (~1.9µA) + LIS3DH @ 10Hz low-power (~11µA) |
| Active (advertising) | ~1.5 mA avg | BLE advertising @ 250-400ms + battery ADC sampling |
| Peak (TX burst) | ~8-10 mA | During BLE transmit events (~1-3ms per event) |

### Component Breakdown (Sleep Mode)

| Component | Current | Notes |
|-----------|---------|-------|
| nRF52832 System ON | ~1.9 µA | RAM retained, RTC running |
| LIS3DH @ 10Hz LP mode | ~6-11 µA | Interrupt engine active, high-pass filter |
| GPIO leakage | <0.1 µA | Negligible with proper configuration |
| **Total Sleep** | **~10-13 µA** | Conservative estimate: 13µA |

### Component Breakdown (Active Mode)

| Component | Current | Notes |
|-----------|---------|-------|
| BLE advertising | ~0.5-1.5 mA | 250-400ms interval, connectable/scannable |
| LIS3DH motion monitoring | ~11 µA | Same as sleep, always running |
| ADC sampling (periodic) | ~0.3 mA | Brief spikes every 30 seconds |
| CPU overhead | ~0.1-0.3 mA | Processing interrupts, state management |
| **Total Active** | **~1.0-2.0 mA** | Conservative estimate: 1.5mA |

---

## Battery Life Calculations

### Supported Power Sources

| Battery Type | Nominal Voltage | Typical Capacity | Effective Capacity* |
|--------------|-----------------|------------------|---------------------|
| CR2032 | 3.0V | 225 mAh | ~180 mAh |
| 2× AAA Alkaline | 3.0V (1.5V × 2) | 1,200 mAh | ~1,000 mAh |
| 2× AA Alkaline | 3.0V (1.5V × 2) | 2,850 mAh | ~2,400 mAh |
| 2× AA Lithium | 3.0V (1.5V × 2) | 3,500 mAh | ~3,000 mAh |

*Effective capacity accounts for ~15-20% derating due to voltage cutoff (2.0V minimum), temperature, and discharge curve.

### Usage Pattern Definitions

| Pattern | Description | Daily Active Time | Trips/Day |
|---------|-------------|-------------------|-----------|
| **Parked Vehicle** | Garaged/parked, minimal vibration | ~5-15 min | 0-1 |
| **Weekend Rider** | Motorcycle/classic car, weekend use only | ~2-4 hours | 1-2 (weekends only) |
| **Commuter** | Daily work commute, 2× per day ~1h each | ~2-2.5 hours | 2 |
| **Daily Driver** | Multiple short trips throughout day | ~3-4 hours | 4-6 |
| **Heavy Use** | Delivery/rideshare, constant use | ~8-12 hours | 10-20 |

### Battery Life Formula

```
Total Daily Consumption (mAh/day) = (I_sleep × T_sleep) + (I_active × T_active)

Where:
  I_sleep  = 0.013 mA (13 µA)
  I_active = 1.5 mA (conservative average)
  T_sleep  = 24 - T_active (hours)
  T_active = Active hours per day + (trips × timeout / 3600)
  timeout  = 60 seconds (advertising continues after last motion)

Battery Life (days) = Effective_Capacity / Daily_Consumption
```

### Detailed Lifetime Estimates

#### CR2032 (180 mAh effective)

| Usage Pattern | Active Hours/Day | Daily Draw (mAh) | Estimated Life |
|---------------|------------------|------------------|----------------|
| Parked Vehicle | 0.25h | 0.69 | **261 days** (~9 months) |
| Weekend Rider | 0.57h (avg)* | 1.14 | **158 days** (~5 months) |
| Commuter (2×/day) | 2.03h | 3.33 | **54 days** (~2 months) |
| Daily Driver | 3.1h | 4.95 | **36 days** (~5 weeks) |
| Heavy Use | 8h | 12.27 | **15 days** (~2 weeks) |

*Weekend rider averaged: (4h × 2 days + 0h × 5 days) / 7 = 1.14h/day

#### 2× AAA Alkaline (1,000 mAh effective)

| Usage Pattern | Active Hours/Day | Daily Draw (mAh) | Estimated Life |
|---------------|------------------|------------------|----------------|
| Parked Vehicle | 0.25h | 0.69 | **1,449 days** (~4 years) |
| Weekend Rider | 0.57h (avg) | 1.14 | **877 days** (~2.4 years) |
| Commuter (2×/day) | 2.03h | 3.33 | **300 days** (~10 months) |
| Daily Driver | 3.1h | 4.95 | **202 days** (~7 months) |
| Heavy Use | 8h | 12.27 | **82 days** (~3 months) |

#### 2× AA Alkaline (2,400 mAh effective)

| Usage Pattern | Active Hours/Day | Daily Draw (mAh) | Estimated Life |
|---------------|------------------|------------------|----------------|
| Parked Vehicle | 0.25h | 0.69 | **3,478 days** (~9.5 years)† |
| Weekend Rider | 0.57h (avg) | 1.14 | **2,105 days** (~5.8 years)† |
| Commuter (2×/day) | 2.03h | 3.33 | **721 days** (~2 years) |
| Daily Driver | 3.1h | 4.95 | **485 days** (~1.3 years) |
| Heavy Use | 8h | 12.27 | **196 days** (~6 months) |

†Note: Alkaline batteries self-discharge ~2-3% per year. Actual life limited to ~5-7 years maximum regardless of load.

#### 2× AA Lithium (3,000 mAh effective)

| Usage Pattern | Active Hours/Day | Daily Draw (mAh) | Estimated Life |
|---------------|------------------|------------------|----------------|
| Parked Vehicle | 0.25h | 0.69 | **4,348 days** (~12 years)† |
| Weekend Rider | 0.57h (avg) | 1.14 | **2,632 days** (~7 years)† |
| Commuter (2×/day) | 2.03h | 3.33 | **901 days** (~2.5 years) |
| Daily Driver | 3.1h | 4.95 | **606 days** (~1.7 years) |
| Heavy Use | 8h | 12.27 | **245 days** (~8 months) |

†Lithium batteries have ~1% annual self-discharge, supporting longer shelf life. Practical limit ~10 years.

### Commuter Scenario Deep Dive (2× daily, ~1h each)

This is a common use case: daily work commute with two trips.

**Calculation breakdown:**
- Morning commute: 1 hour active
- Evening commute: 1 hour active  
- Post-trip timeout: 60 seconds × 2 trips = 2 minutes
- **Total active time**: 2h 2min/day ≈ 2.03 hours

**Daily energy consumption:**
```
Sleep:   (24 - 2.03)h × 0.013mA = 0.286 mAh
Active:  2.03h × 1.5mA           = 3.045 mAh
─────────────────────────────────────────────
Total:                           = 3.33 mAh/day
```

| Battery Type | Capacity | Estimated Life |
|--------------|----------|----------------|
| CR2032 | 180 mAh | **54 days** (~2 months) |
| 2× AAA | 1,000 mAh | **300 days** (~10 months) |
| 2× AA | 2,400 mAh | **721 days** (~2 years) |
| 2× AA Lithium | 3,000 mAh | **901 days** (~2.5 years) |

### Temperature Impact

Battery performance degrades significantly in extreme temperatures:

| Temperature | Capacity Factor | Notes |
|-------------|-----------------|-------|
| -20°C to -10°C | 50-70% | Severe cold reduces capacity |
| -10°C to 0°C | 70-85% | Cold weather impact |
| 0°C to 25°C | 85-100% | Optimal operating range |
| 25°C to 45°C | 90-100% | Warm, generally fine |
| >45°C | Reduced lifespan | Accelerated self-discharge |

**Recommendation**: For vehicles parked outdoors in extreme climates, use **lithium primary cells** (2× AA Lithium) which maintain 90%+ capacity down to -40°C.

### Battery Selection Guide

| Use Case | Recommended | Rationale |
|----------|-------------|-----------|
| **Motorcycle (Weekend)** | 2× AAA | Compact, 2+ years life |
| **Daily Commuter** | 2× AA | Best balance of size/life (~2 years) |
| **Delivery Vehicle** | 2× AA Lithium | Heavy use, ~8 months |
| **Garage Queen** | 2× AA | Multi-year life, low self-discharge concern |
| **Extreme Cold Climate** | 2× AA Lithium | Maintains capacity at low temps |
| **Compact Installation** | CR2032 | Small size, replace every 2-9 months |

### Quick Reference: Months Until Replacement

|  | CR2032 | 2× AAA | 2× AA | 2× AA Li |
|--|--------|--------|-------|----------|
| **Parked** | 9 mo | 48 mo | 48 mo* | 48 mo* |
| **Weekend** | 5 mo | 29 mo | 48 mo* | 48 mo* |
| **Commuter** | 2 mo | 10 mo | 24 mo | 30 mo |
| **Daily** | 1 mo | 7 mo | 16 mo | 20 mo |
| **Heavy** | 0.5 mo | 3 mo | 6 mo | 8 mo |

*Limited by battery self-discharge, not device consumption

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
