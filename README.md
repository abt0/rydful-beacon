# Rydful Beacon

<img src="hardware/pcb-rec.png" alt="Rydful Beacon PCB" width="200" align="left" style="margin-right: 20px; margin-bottom: 10px;">

A motion-triggered BLE beacon application for Nordic nRF52 microcontrollers using the Zephyr RTOS. The device remains in sleep mode until motion is detected via an LIS3DH accelerometer, then starts BLE advertising to enable presence detection.

Designed for Rydful App: https://rydful.com

<br clear="left">

## Features

- **Hardware Interrupt-Driven Wake** – LIS3DH accelerometer's internal motion engine triggers MCU wake via GPIO interrupt (P0.02/INT1)
- **Ultra-Low Power Sleep** – MCU sleeps indefinitely with `k_sem_take(K_FOREVER)` until motion interrupt occurs (~3.9 µA total idle current)
- **Motion-Triggered Advertising** – BLE advertising starts only when motion exceeds hardware threshold (48mg default)
- **High-Pass Filtered Detection** – Gravity filtered out at 1Hz ODR in hardware, only acceleration changes trigger interrupt
- **Configurable Timeouts** – Advertising stops after configurable period of no motion (default: 30 seconds)
- **Battery Voltage Monitoring** – ADC-based battery voltage measurement with percentage calculation, included in BLE advertising data (updated every 5 minutes)
- **Direct I2C Register Access** – Bypasses Zephyr sensor framework for reliable interrupt configuration
- **Android CDM Compatible** – 250-400ms connectable/scannable advertising interval for reliable companion app discovery

## Hardware Requirements

- **MCU**: Nordic nRF52832 (nRF52 DK)
- **Accelerometer**: LIS3DH or LIS2DH (I2C interface)
- **Battery**: 2.0-3.6V input (CR2032, 2× AAA/AA alkaline or lithium)

### Wiring (nRF52 DK with LIS3DH)

| LIS3DH Pin | nRF52 DK Pin | Description |
|------------|--------------|-------------|
| SDA        | P0.26        | I2C Data    |
| SCL        | P0.27        | I2C Clock   |
| INT1       | P0.02        | Motion interrupt (required for low-power wake) |
| -          | P0.03 (AIN1) | Battery voltage input (via voltage divider if >3.6V) |

## Project Structure

```
rydful-beacon/
├── CMakeLists.txt               # Build configuration
├── prj.conf                     # Zephyr project configuration
├── hardware/                    # Hardware description
├── src/
│   └── main.c                   # Main application source
└── boards/
    ├── nrf52dk_nrf52832.overlay # nRF52 DK device tree overlay
    └── rydful_custom/           # Custom PCB board definition
          ├── rydful_custom.dts           # Device tree
          ├── rydful_custom-pinctrl.dtsi  # Pin control
          ├── rydful_custom_defconfig     # Default Kconfig
          ├── board.yaml                  # Board metadata
          ├── Kconfig.rydful_custom       # Board Kconfig
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
| `HW_MOTION_THRESHOLD_MG` | 32 mg | Acceleration threshold for motion detection |
| `HW_MOTION_DURATION` | 2 samples | Required samples above threshold (debounce) |
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
│  • LIS3DH: Hardware threshold interrupt (>32mg) @ 1Hz       │
│  • nRF52: System ON sleep with LFXO (~1.8 µA)               │
│  • Total: ~3.9 µA                                           │
└──────────────────────┬──────────────────────────────────────┘
                       │ Motion exceeds threshold (~2s latency)
                       ▼
┌─────────────────────────────────────────────────────────────┐
│  ACTIVE MODE                                                │
│  • BLE advertising (connectable, 250-400ms, -12dBm TX)      │
│  • Each motion interrupt resets 30-second timeout           │
│  • Total: ~55 µA                                            │
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
Motion threshold: 32 mg
Motion duration: 2 samples
No-motion timeout: 30 seconds
Device name: Rydful_Beacon
==========================================
LIS3DH detected (WHO_AM_I: 0x33)
HW motion OK: threshold=32mg duration=2 ODR=1Hz
Battery ADC initialized (AIN1/P0.03, gain=1/6, ref=0.6V)
[BATT] Voltage: 3000 mV, Percentage: 100% (OK)
Bluetooth ready
Ready - move device to start advertising
Entering sleep (waiting for motion)...
Motion detected - waking up!
>>> BLE advertising STARTED (motion detected)
```

## Power Consumption

*Note: All estimates are for the production PCB (nRF52832 + LIS3DHTR + passives only, no LED).*

### Current Draw by State

| State | Current | Description |
|-------|---------|-------------|
| Sleep | ~3.9 µA | nRF52 System ON (~1.8µA with LFXO) + LIS3DH @ 1Hz (~2µA) |
| Active (advertising) | ~55 µA avg | BLE connectable advertising @ 250-400ms, -12dBm TX |
| Peak (TX burst) | ~7.5 mA | During BLE transmit events @ -12dBm (~380µs per event) |

### Component Breakdown (Sleep Mode)

| Component | Current | Notes |
|-----------|---------|-------|
| nRF52832 System ON (LFXO) | ~1.8 µA | RAM retained, RTC running, external 32.768kHz crystal |
| LIS3DHTR @ 1Hz LP mode | ~2 µA | Interrupt engine active, HP filter, 1s wake latency |
| GPIO leakage | ~0.1 µA | Minimal with proper configuration |
| **Total Sleep** | **~3.9 µA** | |

### Component Breakdown (Active Mode)

| Component | Current | Notes |
|-----------|---------|-------|
| BLE advertising | ~40 µA | 250-400ms interval, connectable/scannable, -12dBm TX (duty-cycled avg) |
| LIS3DHTR @ 1Hz | ~2 µA | Same ODR as sleep |
| ADC sampling (periodic) | ~1 µA avg | Brief spikes every 5 minutes |
| CPU overhead | ~10-12 µA | Processing interrupts, state management |
| **Total Active** | **~55 µA** | Conservative estimate: 0.055mA |

### Power Optimizations Applied

| Optimization | Impact | Trade-off |
|--------------|--------|-----------|
| External 32.768kHz LFXO | ~70% MCU sleep reduction vs RC oscillator | None (hardware present) |
| Reduced TX power (-12dBm) | ~25% peak TX reduction, minimal avg impact | Tight 5m discovery range |
| LIS3DH 1Hz ODR | ~64% sensor sleep reduction | 1 second wake latency |
| 30s no-motion timeout | Minimizes active time | Slightly faster sleep transition |
| 5-minute battery updates | Negligible impact (<1 µA) | Less frequent battery status |

---

## Battery Life Calculations

*Production PCB with power optimizations: 1Hz accelerometer ODR, 30s timeout, 5-min battery updates.*

### Supported Power Sources

| Battery Type | Nominal Voltage | Typical Capacity | Effective Capacity* |
|--------------|-----------------|------------------|---------------------|
| CR2032 | 3.0V | 225 mAh | ~150 mAh** |
| 2× AAA Alkaline | 3.0V (1.5V × 2) | 1,200 mAh | ~1,000 mAh |
| 2× AA Alkaline | 3.0V (1.5V × 2) | 2,850 mAh | ~2,400 mAh |
| 2× AA Lithium | 3.0V (1.5V × 2) | 3,500 mAh | ~3,000 mAh |

*Effective capacity accounts for ~15-20% derating due to voltage cutoff (2.0V minimum), temperature, and discharge curve.

**CR2032 Pulse Current Derating:** CR2032 batteries experience significant capacity reduction under pulse loads. While average current is low (~55 µA active), the 7.5 mA TX peaks (though brief, ~380µs every 325ms) cause additional derating. Effective capacity reduced from nominal 225 mAh to ~150 mAh for this application. For comparison: at 0.5 mA continuous draw CR2032 provides ~240 mAh, but at 3.0 mA only ~155 mAh remains available.

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
  I_sleep  = 0.0039 mA (3.9 µA with external LFXO)
  I_active = 0.055 mA (duty-cycled average, -12dBm TX power)
  T_sleep  = 24 - T_active (hours)
  T_active = Active hours per day + (trips × timeout / 3600)
  timeout  = 30 seconds (advertising continues after last motion)

Battery Life (days) = Effective_Capacity / Daily_Consumption
```

### Detailed Lifetime Estimates

#### CR2032 (150 mAh effective, pulse-derated)

| Usage Pattern | Active Hours/Day | Daily Draw (mAh) | Estimated Life |
|---------------|------------------|------------------|----------------|
| Parked Vehicle | 0.26h | 0.11 | **1,402 days** (~3.8 years)† |
| Weekend Rider | 0.58h (avg)* | 0.12 | **1,220 days** (~3.3 years)† |
| Commuter (2×/day) | 2.02h | 0.20 | **761 days** (~2.1 years) |
| Daily Driver | 3.04h | 0.25 | **602 days** (~1.6 years) |
| Heavy Use | 8.13h | 0.51 | **295 days** (~9.7 months) |

*Weekend rider averaged over 7 days
†CR2032 self-discharge ~10-15% per year; practical life 1.5-2 years for commuter/daily use due to pulse derating and environmental factors

#### 2× AAA Alkaline (1,000 mAh effective)

| Usage Pattern | Active Hours/Day | Daily Draw (mAh) | Estimated Life |
|---------------|------------------|------------------|----------------|
| Parked Vehicle | 0.26h | 0.11 | **9,346 days** (~25.6 years)† |
| Weekend Rider | 0.58h (avg) | 0.12 | **8,130 days** (~22.3 years)† |
| Commuter (2×/day) | 2.02h | 0.20 | **5,076 days** (~13.9 years)† |
| Daily Driver | 3.04h | 0.25 | **4,016 days** (~11.0 years)† |
| Heavy Use | 8.13h | 0.51 | **1,965 days** (~5.4 years) |

†Limited by battery self-discharge (~2-3% per year), practical limit ~5-7 years.

#### 2× AA Alkaline (2,400 mAh effective)

| Usage Pattern | Active Hours/Day | Daily Draw (mAh) | Estimated Life |
|---------------|------------------|------------------|----------------|
| Parked Vehicle | 0.26h | 0.11 | **22,430 days** (~61 years)† |
| Weekend Rider | 0.58h (avg) | 0.12 | **19,512 days** (~53 years)† |
| Commuter (2×/day) | 2.02h | 0.20 | **12,183 days** (~33 years)† |
| Daily Driver | 3.04h | 0.25 | **9,639 days** (~26 years)† |
| Heavy Use | 8.13h | 0.51 | **4,716 days** (~12.9 years)† |

†Alkaline batteries self-discharge ~2-3% per year. Actual life limited to ~5-7 years maximum regardless of load.

#### 2× AA Lithium (3,000 mAh effective)

| Usage Pattern | Active Hours/Day | Daily Draw (mAh) | Estimated Life |
|---------------|------------------|------------------|----------------|
| Parked Vehicle | 0.26h | 0.11 | **28,037 days** (~76 years)† |
| Weekend Rider | 0.58h (avg) | 0.12 | **24,390 days** (~66 years)† |
| Commuter (2×/day) | 2.02h | 0.20 | **15,228 days** (~41 years)† |
| Daily Driver | 3.04h | 0.25 | **12,048 days** (~33 years)† |
| Heavy Use | 8.13h | 0.51 | **5,895 days** (~16.1 years)† |

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
Sleep:   (24 - 2.02)h × 0.0039mA = 0.086 mAh
Active:  2.02h × 0.055mA         = 0.111 mAh
─────────────────────────────────────────────
Total:                           = 0.197 mAh/day
```

| Battery Type | Capacity | Estimated Life | Practical Reality |
|--------------|----------|----------------|-------------------|
| CR2032 | 150 mAh | **761 days** (~2.1 years) | 1.5-2 years typical |
| 2× AAA | 1,000 mAh | **5,076 days** (~13.9 years)† | 5-7 years (self-discharge limit) |
| 2× AA | 2,400 mAh | **12,183 days** (~33 years)† | 5-7 years (self-discharge limit) |
| 2× AA Lithium | 3,000 mAh | **15,228 days** (~41 years)† | 10+ years (self-discharge limit) |

†Limited by battery self-discharge, not device consumption

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
| **Motorcycle (Weekend)** | 2× AAA | AAA: 5-7 years (self-discharge limited), better than CR2032 for longevity |
| **Daily Commuter** | 2× AAA | **5-7 years** (self-discharge limited), compact form factor |
| **Daily Commuter (extended)** | 2× AA | **5-7 years** (self-discharge limited), best longevity |
| **Delivery Vehicle** | 2× AA or 2× AA Lithium | AA: ~5 years, Lithium: 10+ years |
| **Garage Queen** | 2× AAA | Multi-year life, minimal replacement (AAA better than CR2032) |
| **Extreme Cold Climate** | 2× AA Lithium | Maintains capacity at low temps, 10+ years |
| **Compact Installation** | CR2032 | Small size, **1.5-2 years typical** (pulse current limits lifespan) |

### Quick Reference: Months Until Replacement

|  | CR2032** | 2× AAA | 2× AA | 2× AA Li |
|--|--------|--------|-------|----------|
| **Parked** | 46 mo | 60+ mo* | 60+ mo* | 120+ mo* |
| **Weekend** | 40 mo | 60+ mo* | 60+ mo* | 120+ mo* |
| **Commuter** | **25 mo** | 60+ mo* | 60+ mo* | 120+ mo* |
| **Daily** | 20 mo | 60+ mo* | 60+ mo* | 120+ mo* |
| **Heavy** | 10 mo | 64 mo* | 60+ mo* | 120+ mo* |

*Limited by battery self-discharge (~5-7 years for alkaline, ~10 years for lithium), not device consumption
**CR2032 estimates account for pulse current derating; real-world life often 1.5-2 years for commuter/daily use due to environmental factors

SPDX-License-Identifier: Apache-2.0
