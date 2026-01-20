# Rydful Beacon

A motion-triggered BLE beacon application for Nordic nRF52 microcontrollers using the Zephyr RTOS. The device remains in sleep mode until motion is detected via an LIS3DH accelerometer, then starts BLE advertising to enable presence detection.

Designed for Rydful App: https://rydful.com

## Features

- **Hardware-Driven Motion Detection** – LIS3DH accelerometer handles motion detection internally at 1Hz, MCU only wakes on threshold events (~1s latency)
- **Ultra-Low Power Sleep** – CPU sleeps indefinitely until real motion occurs (~2.5 µA total idle current with external LFXO)
- **Motion-Triggered Advertising** – BLE advertising starts only when motion exceeds hardware threshold
- **Battery Monitoring** – Battery voltage measurement with percentage calculation, broadcast via BLE manufacturer data
- **Configurable Timeouts** – Advertising stops after configurable period of no motion (default: 30 seconds)
- **High-Pass Filtered Detection** – Gravity is filtered out, only acceleration changes trigger wake
- **Android CDM Compatible** – 250-400ms advertising interval for reliable companion app discovery

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
| `NO_MOTION_TIMEOUT_SEC` | 30 | Seconds of no motion before stopping advertising |

The LIS3DH operates at **1Hz ODR** in low-power mode (~2µA) and uses its internal high-pass filter to remove gravity, so only acceleration *changes* trigger the interrupt. Wake latency is ~1 second.

### Battery Monitoring

| Parameter | Default | Description |
|-----------|---------|-------------|
| `BATTERY_LOW_THRESHOLD_MV` | 2000 mV | 0% battery threshold |
| `BATTERY_FULL_MV` | 3000 mV | 100% battery threshold |
| `BATTERY_UPDATE_INTERVAL_SEC` | 300 | Battery status update interval (5 minutes) |

### BLE Configuration

- **Device Name**: `Rydful_Beacon` (configurable via `CONFIG_BT_DEVICE_NAME`)
- **Service UUID**: `0xFEAA` (16-bit)
- **Manufacturer ID**: `0xFFFF` (development/testing)
- **Advertising Interval**: 250-400 ms (required for Android CDM compatibility)
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
│  • LIS3DH: Hardware threshold interrupt (>48mg) @ 1Hz       │
│  • nRF52: System ON sleep with LFXO (~0.5 µA)               │
│  • Total: ~2.5 µA                                           │
└──────────────────────┬──────────────────────────────────────┘
                       │ Motion exceeds threshold (~1s latency)
                       ▼
┌─────────────────────────────────────────────────────────────┐
│  ACTIVE MODE                                                │
│  • BLE advertising (connectable, 250-400ms, -12dBm TX)      │
│  • Each motion interrupt resets 30-second timeout           │
│  • Total: ~0.5-0.6 mA                                       │
└──────────────────────┬──────────────────────────────────────┘
                       │ No motion for 30 seconds
                       ▼
              Back to SLEEP MODE
```

### Usage

1. **Power on** – Initialization sequence
2. **Sleep mode** – No BLE advertising, ultra-low power (~5 µA)
3. **Motion detected** – BLE advertising starts (~1 second wake latency)
4. **Continuous motion** – Each motion event resets the 30-second timeout
5. **No motion** – After 30 seconds of no motion, advertising stops

### Monitoring

Connect via RTT console (production PCB) or serial (115200 baud) to view logs:

```
==========================================
  Motion-Triggered BLE Beacon
==========================================
Mode: Hardware motion detection
Motion threshold: 48 mg
Motion duration: 1 samples
No-motion timeout: 30 seconds
Battery update interval: 300 seconds
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

*Note: All estimates are for the production PCB (nRF52832 + LIS3DHTR + passives only, no LED).*

### Current Draw by State

| State | Current | Description |
|-------|---------|-------------|
| Sleep | ~2.5 µA | nRF52 System ON (~0.5µA with LFXO) + LIS3DH @ 1Hz (~2µA) |
| Active (advertising) | ~0.5-0.6 mA avg | BLE connectable advertising @ 250-400ms, -12dBm TX |
| Peak (TX burst) | ~3-4 mA | During BLE transmit events @ -12dBm (~1-3ms per event) |

### Component Breakdown (Sleep Mode)

| Component | Current | Notes |
|-----------|---------|-------|
| nRF52832 System ON (LFXO) | ~0.5 µA | RAM retained, RTC running, external 32.768kHz crystal |
| LIS3DHTR @ 1Hz LP mode | ~2 µA | Interrupt engine active, HP filter, 1s wake latency |
| GPIO leakage | <0.1 µA | Minimal with proper configuration |
| **Total Sleep** | **~2.5 µA** | |

### Component Breakdown (Active Mode)

| Component | Current | Notes |
|-----------|---------|-------|
| BLE advertising | ~0.45-0.55 mA | 250-400ms interval, connectable/scannable, -12dBm TX |
| LIS3DHTR @ 1Hz | ~2 µA | Same ODR as sleep |
| ADC sampling (periodic) | ~1 µA avg | Brief spikes every 5 minutes |
| CPU overhead | ~0.05 mA | Processing interrupts, state management |
| **Total Active** | **~0.7-0.8 mA** | Conservative estimate: 0.75mA |

### Power Optimizations Applied

| Optimization | Impact | Trade-off |
|--------------|--------|-----------|
| External 32.768kHz LFXO | ~80% MCU sleep reduction | None (hardware present) |
| Reduced TX power (-12dBm) | ~35% active reduction | Tight 5m discovery range |
| LIS3DH 1Hz ODR | ~64% sensor sleep reduction | 1 second wake latency |
| 30s no-motion timeout | ~3% overall | Slightly faster sleep transition |
| 5-minute battery updates | ~1% overall | Less frequent battery status |

---

## Battery Life Calculations

*Production PCB with power optimizations: 1Hz accelerometer ODR, 30s timeout, 5-min battery updates.*

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
| **Commuter** | Daily work commute, 2× per day ~1h each | ~2 hours | 2 |
| **Daily Driver** | Multiple short trips throughout day | ~3 hours | 4-6 |
| **Heavy Use** | Delivery/rideshare, constant use | ~8-12 hours | 10-20 |

### Battery Life Formula

```
Total Daily Consumption (mAh/day) = (I_sleep × T_sleep) + (I_active × T_active)

Where:
  I_sleep  = 0.0025 mA (2.5 µA with external LFXO)
  I_active = 0.55 mA (conservative average, -12dBm TX power)
  T_sleep  = 24 - T_active (hours)
  T_active = Active hours per day + (trips × timeout / 3600)
  timeout  = 30 seconds (advertising continues after last motion)

Battery Life (days) = Effective_Capacity / Daily_Consumption
```

### Detailed Lifetime Estimates

#### CR2032 (180 mAh effective)

| Usage Pattern | Active Hours/Day | Daily Draw (mAh) | Estimated Life |
|---------------|------------------|------------------|----------------|
| Parked Vehicle | 0.26h | 0.20 | **890 days** (~29 months) |
| Weekend Rider | 0.58h (avg)* | 0.38 | **477 days** (~16 months) |
| Commuter (2×/day) | 2.02h | 1.17 | **154 days** (~5 months) |
| Daily Driver | 3.04h | 1.72 | **104 days** (~3.5 months) |
| Heavy Use | 8.13h | 4.51 | **40 days** (~6 weeks) |

*Weekend rider averaged over 7 days

#### 2× AAA Alkaline (1,000 mAh effective)

| Usage Pattern | Active Hours/Day | Daily Draw (mAh) | Estimated Life |
|---------------|------------------|------------------|----------------|
| Parked Vehicle | 0.26h | 0.20 | **4,940 days** (~13.5 years)† |
| Weekend Rider | 0.58h (avg) | 0.38 | **2,650 days** (~7.3 years)† |
| Commuter (2×/day) | 2.02h | 1.17 | **858 days** (~28 months) |
| Daily Driver | 3.04h | 1.72 | **580 days** (~19 months) |
| Heavy Use | 8.13h | 4.51 | **222 days** (~7.3 months) |

†Limited by battery self-discharge (~2-3% per year), practical limit ~5-7 years.

#### 2× AA Alkaline (2,400 mAh effective)

| Usage Pattern | Active Hours/Day | Daily Draw (mAh) | Estimated Life |
|---------------|------------------|------------------|----------------|
| Parked Vehicle | 0.26h | 0.20 | **11,860 days** (~32 years)† |
| Weekend Rider | 0.58h (avg) | 0.38 | **6,360 days** (~17 years)† |
| Commuter (2×/day) | 2.02h | 1.17 | **2,058 days** (~5.6 years) |
| Daily Driver | 3.04h | 1.72 | **1,392 days** (~3.8 years) |
| Heavy Use | 8.13h | 4.51 | **532 days** (~18 months) |

†Alkaline batteries self-discharge ~2-3% per year. Actual life limited to ~5-7 years maximum regardless of load.

#### 2× AA Lithium (3,000 mAh effective)

| Usage Pattern | Active Hours/Day | Daily Draw (mAh) | Estimated Life |
|---------------|------------------|------------------|----------------|
| Parked Vehicle | 0.26h | 0.20 | **14,826 days** (~41 years)† |
| Weekend Rider | 0.58h (avg) | 0.38 | **7,946 days** (~22 years)† |
| Commuter (2×/day) | 2.02h | 1.17 | **2,573 days** (~7.0 years) |
| Daily Driver | 3.04h | 1.72 | **1,740 days** (~4.8 years) |
| Heavy Use | 8.13h | 4.51 | **665 days** (~22 months) |

†Lithium batteries have ~1% annual self-discharge. Practical limit ~10 years.

### Commuter Scenario Deep Dive (2× daily, ~1h each)

This is a common use case: daily work commute with two trips.

**Calculation breakdown:**
- Morning commute: 1 hour active
- Evening commute: 1 hour active
- Post-trip timeout: 30 seconds × 2 trips = 1 minute
- **Total active time**: 2h 1min/day ≈ 2.02 hours

**Daily energy consumption:**
```
Sleep:   (24 - 2.02)h × 0.0025mA = 0.055 mAh
Active:  2.02h × 0.55mA          = 1.111 mAh
─────────────────────────────────────────────
Total:                           = 1.17 mAh/day
```

| Battery Type | Capacity | Estimated Life |
|--------------|----------|----------------|
| CR2032 | 180 mAh | **154 days** (~5 months) |
| 2× AAA | 1,000 mAh | **858 days** (~28 months) |
| 2× AA | 2,400 mAh | **2,058 days** (~5.6 years) |
| 2× AA Lithium | 3,000 mAh | **2,573 days** (~7.0 years) |

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
| **Motorcycle (Weekend)** | 2× AAA | Compact, 5+ years (self-discharge limited) |
| **Daily Commuter** | 2× AAA | **21 months**, compact form factor |
| **Daily Commuter (extended)** | 2× AA | **4+ years**, best longevity |
| **Delivery Vehicle** | 2× AA Lithium | Heavy use, ~16 months |
| **Garage Queen** | 2× AAA | Multi-year life, compact |
| **Extreme Cold Climate** | 2× AA Lithium | Maintains capacity at low temps |
| **Compact Installation** | CR2032 | Small size, replace every 4-24 months |

### Quick Reference: Months Until Replacement

|  | CR2032 | 2× AAA | 2× AA | 2× AA Li |
|--|--------|--------|-------|----------|
| **Parked** | 29 mo | 60 mo* | 60 mo* | 60 mo* |
| **Weekend** | 16 mo | 60 mo* | 60 mo* | 60 mo* |
| **Commuter** | 5 mo | **28 mo** | 67 mo | 84 mo |
| **Daily** | 3.5 mo | 19 mo | 46 mo | 58 mo |
| **Heavy** | 1.3 mo | 7 mo | 18 mo | 22 mo |

*Limited by battery self-discharge (~5-7 years for alkaline, ~10 years for lithium), not device consumption

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
