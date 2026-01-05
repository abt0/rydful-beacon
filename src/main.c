/*
 * Motion-Triggered BLE Beacon for nRF52 with LIS3DH Accelerometer
 *
 * - Starts in sleep mode (no BLE advertising)
 * - Detects motion via accelerometer
 * - Starts BLE advertising when motion detected
 * - Stops advertising after 10 seconds of no motion
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(ble_beacon, LOG_LEVEL_INF);

/* Motion detection configuration for car movement */
/* Note: At 1Hz ODR, each sample is 1 second apart - thresholds tuned for sensitivity */
#define MOTION_THRESHOLD          1.0f    /* Absolute delta threshold (after EMA filtering) */
#define NOISE_DEADBAND            0.9f    /* Ignore deltas below this (sensor quantization noise) */
#define VARIANCE_THRESHOLD        1.8f    /* Variance threshold for road vibrations */
#define MIN_MOTION_HITS           5       /* Require 5 hits (5 seconds at 1Hz) */
#define VARIANCE_WINDOW_SIZE      8       /* Number of samples for variance calculation */
#define EMA_ALPHA                 0.3f    /* EMA smoothing factor (0.3 = moderate smoothing) */
#define NO_MOTION_TIMEOUT_SEC     60      /* Timeout for highway cruising gaps */
#define ACCEL_SAMPLE_INTERVAL_MS  100     /* Sampling interval when advertising (ms) */
#define IDLE_POLL_INTERVAL_MS     2000    /* Slow polling when idle (power saving) */

/* LED feedback configuration */
#define STARTUP_BLINK_COUNT       3       /* Number of blinks on startup */
#define STARTUP_BLINK_MS          100     /* Duration of each startup blink (ms) */

/* Gravity sanity check bounds (m/s²) - squared for comparison */
#define GRAVITY_MIN_MS2_SQ        25.0f   /* (5.0)^2 */
#define GRAVITY_MAX_MS2_SQ        225.0f  /* (15.0)^2 */

/* ADC conversion constants */
#define ADC_REF_MV                3600    /* Full scale voltage (0.6V * 6 gain) */
#define ADC_RESOLUTION            4096    /* 12-bit resolution */

/* ========== LIS2DH Direct Register Configuration ========== */
/* Register addresses */
#define LIS2DH_REG_CTRL_REG1      0x20
#define LIS2DH_REG_CTRL_REG3      0x22
#define LIS2DH_REG_CTRL_REG4      0x23
#define LIS2DH_REG_CTRL_REG5      0x24
#define LIS2DH_REG_STATUS_REG     0x27

/* CTRL_REG3 bits - INT1 configuration */
#define LIS2DH_I1_DRDY1           BIT(4)  /* Data-ready interrupt on INT1 */
#define LIS2DH_I1_AOI1            BIT(6)  /* AOI1 interrupt on INT1 */

/* CTRL_REG1 bits - ODR and axis enable */
#define LIS2DH_ODR_1HZ            (0x01 << 4)  /* 1 Hz output data rate (lowest power) */
#define LIS2DH_ODR_10HZ           (0x02 << 4)  /* 10 Hz output data rate */
#define LIS2DH_ODR_25HZ           (0x03 << 4)  /* 25 Hz output data rate */
#define LIS2DH_XYZ_EN             0x07         /* Enable X, Y, Z axes */
#define LIS2DH_LPEN               BIT(3)       /* Low-power mode enable */

/* INT1 GPIO configuration */
#define INT1_NODE DT_NODELABEL(lis3dh)
#define INT1_GPIO_PORT DT_GPIO_CTLR_BY_IDX(DT_NODELABEL(gpio0), gpios, 0)

/* LIS3DH accelerometer device */
static const struct device *lis3dh_dev;

/* INT1 GPIO for direct hardware interrupt (bypasses broken Zephyr trigger API) */
static const struct gpio_dt_spec int1_gpio = GPIO_DT_SPEC_GET_OR(
	DT_NODELABEL(lis3dh), irq_gpios, {0});
static struct gpio_callback int1_cb_data;

/* Semaphore for data-ready interrupt signaling */
static K_SEM_DEFINE(data_ready_sem, 0, 1);

/* Flag to track if hardware interrupt is available */
static bool hw_interrupt_available;

/* LED for visual feedback (optional - may not exist on custom boards) */
#if DT_NODE_EXISTS(DT_ALIAS(led0))
#define HAS_LED 1
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
#else
#define HAS_LED 0
#endif

/* LED helper to reduce #if HAS_LED repetition */
static inline void set_led(int state)
{
#if HAS_LED
	gpio_pin_set_dt(&led, state);
#else
	ARG_UNUSED(state);
#endif
}

/* BLE advertising configuration */
#define ADV_LOG_INTERVAL_SEC  10  /* Log advertising status every N seconds */

/* Battery monitoring configuration */
#define BATTERY_UPDATE_INTERVAL_SEC  30  /* Update battery in advertising every 30 sec */
#define BATTERY_LOW_THRESHOLD_MV     2200 /* Low battery warning threshold (mV) - 0% */
#define BATTERY_FULL_MV              3000 /* Full battery threshold (mV) - 100% */

/* ========== State Structures ========== */

/* BLE advertising state */
struct ble_state {
	bool is_advertising;
	int64_t last_motion_time;
	int64_t last_adv_log_time;
};
static struct ble_state ble = {
	.is_advertising = false,
	.last_motion_time = 0,
	.last_adv_log_time = 0,
};

/* Battery monitoring state */
struct battery_state {
	uint16_t voltage_mv;
	uint8_t percentage;
	bool is_low;
	int64_t last_update;
};
static struct battery_state battery = {
	.voltage_mv = 0,
	.percentage = 0,
	.is_low = false,
	.last_update = 0,
};

/* Motion detection state - using Welford's online algorithm for variance */
struct motion_state {
	float mean;           /* Running mean */
	float m2;             /* Sum of squared differences from mean */
	int count;            /* Number of samples */
	float prev_mag_sq;    /* Previous magnitude squared (EMA filtered) */
	float ema_mag_sq;     /* EMA filtered magnitude squared */
	bool ema_initialized; /* Flag to initialize EMA on first sample */
	int hit_count;        /* Debounce counter */
};
static struct motion_state motion = {
	.mean = 0.0f,
	.m2 = 0.0f,
	.count = 0,
	.prev_mag_sq = 0.0f,
	.ema_mag_sq = 0.0f,
	.ema_initialized = false,
	.hit_count = 0,
};

/* Semaphore to wait for BT ready */
static K_SEM_DEFINE(bt_ready_sem, 0, 1);

/* Custom 16-bit Service UUID for filtering (uses less space than 128-bit) */
#define CUSTOM_SERVICE_UUID_16  0xFEAA  /* Use a 16-bit UUID */

/* Manufacturer ID (use 0xFFFF for development/testing) */
#define MANUFACTURER_ID  0xFFFF

/* Manufacturer specific data: [Manufacturer ID (2)] [Battery % (1)] [Battery Low (1)] */
static uint8_t mfg_data[] = {
	MANUFACTURER_ID & 0xFF,        /* Manufacturer ID low byte */
	(MANUFACTURER_ID >> 8) & 0xFF, /* Manufacturer ID high byte */
	0x00,                          /* Battery percentage (0-100) */
	0x00,                          /* Battery low flag */
};

/* Advertising data - flags + manufacturer data with battery info */
static struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL,
	              CUSTOM_SERVICE_UUID_16 & 0xFF,
	              (CUSTOM_SERVICE_UUID_16 >> 8) & 0xFF),
	BT_DATA(BT_DATA_MANUFACTURER_DATA, mfg_data, sizeof(mfg_data)),
};

/* Scan response data - device name only */
static struct bt_data sd[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

/*
 * Connectable, scannable advertising with slow interval for power savings.
 * 1000-1500ms interval reduces BLE TX power by ~50% vs fast interval.
 * Still compatible with Android Companion Device Manager (CDM).
 * Test with your companion app to verify discovery time is acceptable.
 */
static struct bt_le_adv_param adv_param = BT_LE_ADV_PARAM_INIT(
	BT_LE_ADV_OPT_CONN | BT_LE_ADV_OPT_SCANNABLE | BT_LE_ADV_OPT_USE_IDENTITY,
	0x0640,  /* 1000ms min (1600 * 0.625ms) */
	0x0960,  /* 1500ms max (2400 * 0.625ms) */
	NULL
);

/* Initialize LED */
static int init_led(void)
{
#if HAS_LED
	if (!gpio_is_ready_dt(&led)) {
		LOG_ERR("LED device not ready");
		return -ENODEV;
	}

	int ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		LOG_ERR("Failed to configure LED: %d", ret);
		return ret;
	}

	LOG_INF("LED initialized");
#else
	LOG_INF("No LED configured");
#endif
	return 0;
}

/* Get LIS3DH device from device tree node label */
#define LIS3DH_NODE DT_NODELABEL(lis3dh)

/* LIS3DH registers for diagnostics */
#define LIS3DH_WHO_AM_I_REG   0x0F
#define LIS3DH_WHO_AM_I_VALUE 0x33
#define LIS3DH_I2C_ADDR       0x19

#ifdef CONFIG_BEACON_DIAGNOSTICS
/* Check if LIS3DH is alive by reading WHO_AM_I register */
static int check_lis3dh_alive(void)
{
	uint8_t who_am_i;
	int ret;

	LOG_INF("--- LIS3DH Diagnostic Check ---");

	/* Step 1: Check if sensor device is ready */
	if (!device_is_ready(lis3dh_dev)) {
		LOG_ERR("[DIAG] LIS3DH device not ready");
		return -ENODEV;
	}
	LOG_INF("[DIAG] Device ready: OK");

	/* Step 2: Read WHO_AM_I register via I2C */
	const struct device *i2c_dev = DEVICE_DT_GET(DT_BUS(LIS3DH_NODE));
	if (!device_is_ready(i2c_dev)) {
		LOG_WRN("[DIAG] I2C bus not ready, skipping WHO_AM_I check");
	} else {
		ret = i2c_reg_read_byte(i2c_dev, LIS3DH_I2C_ADDR, LIS3DH_WHO_AM_I_REG, &who_am_i);
		if (ret < 0) {
			LOG_ERR("[DIAG] Failed to read WHO_AM_I: %d", ret);
			return ret;
		}
		if (who_am_i == LIS3DH_WHO_AM_I_VALUE) {
			LOG_INF("[DIAG] WHO_AM_I: 0x%02X (expected 0x33): OK", who_am_i);
		} else {
			LOG_ERR("[DIAG] WHO_AM_I: 0x%02X (expected 0x33): MISMATCH!", who_am_i);
			return -EINVAL;
		}
	}

	/* Step 3: Try to fetch acceleration data */
	ret = sensor_sample_fetch(lis3dh_dev);
	if (ret < 0) {
		LOG_ERR("[DIAG] Sample fetch failed: %d", ret);
		return ret;
	}
	LOG_INF("[DIAG] Sample fetch: OK");

	/* Step 4: Read acceleration values */
	struct sensor_value accel[3];
	ret = sensor_channel_get(lis3dh_dev, SENSOR_CHAN_ACCEL_XYZ, accel);
	if (ret < 0) {
		LOG_ERR("[DIAG] Channel read failed: %d", ret);
		return ret;
	}

	float x = sensor_value_to_float(&accel[0]);
	float y = sensor_value_to_float(&accel[1]);
	float z = sensor_value_to_float(&accel[2]);
	/* Use squared magnitude to avoid sqrt - compare against squared bounds */
	float mag_sq = x * x + y * y + z * z;

	LOG_INF("[DIAG] Accel X: %.2f m/s²", (double)x);
	LOG_INF("[DIAG] Accel Y: %.2f m/s²", (double)y);
	LOG_INF("[DIAG] Accel Z: %.2f m/s²", (double)z);
	LOG_INF("[DIAG] Magnitude^2: %.2f (expect ~96 if stationary)", (double)mag_sq);

	/* Sanity check: magnitude^2 should be roughly 1g^2 (~96 m²/s⁴) when stationary */
	if (mag_sq < GRAVITY_MIN_MS2_SQ || mag_sq > GRAVITY_MAX_MS2_SQ) {
		LOG_WRN("[DIAG] Magnitude unusual - sensor may need calibration");
	} else {
		LOG_INF("[DIAG] Gravity check: OK");
	}

	LOG_INF("--- LIS3DH is ALIVE and working! ---");
	return 0;
}
#endif /* CONFIG_BEACON_DIAGNOSTICS */

/* Initialize LIS3DH accelerometer */
static int init_lis3dh(void)
{
#if DT_NODE_EXISTS(LIS3DH_NODE)
	lis3dh_dev = DEVICE_DT_GET(LIS3DH_NODE);

	if (!device_is_ready(lis3dh_dev)) {
		LOG_ERR("LIS3DH device %s is not ready", lis3dh_dev->name);
		return -ENODEV;
	}

	LOG_INF("LIS3DH initialized: %s", lis3dh_dev->name);
	return 0;
#else
	LOG_ERR("LIS3DH node not found in device tree - check overlay file");
	return -ENODEV;
#endif
}

/* ========== Battery Voltage Measurement ========== */

/*
 * ADC for battery voltage measurement.
 * Note: nRF52832 does NOT support direct VDD measurement via SAADC.
 * This feature requires either:
 * - nRF52840/nRF54 (has NRF_SAADC_VDD input)
 * - External voltage divider on an AIN pin
 * For nRF52 DK without external circuitry, battery monitoring is disabled.
 */
#if defined(CONFIG_ADC_NRFX_SAADC) && defined(NRF_SAADC_VDD)
#define BATTERY_ADC_SUPPORTED 1
#else
#define BATTERY_ADC_SUPPORTED 0
#endif

#if BATTERY_ADC_SUPPORTED
#define ADC_NODE DT_NODELABEL(adc)
#define ADC_CHANNEL_ID 0
#define ADC_RESOLUTION_BITS 12

static const struct device *adc_dev;
static int16_t adc_buffer;
static bool adc_available;

static const struct adc_channel_cfg channel_cfg = {
	.gain = ADC_GAIN_1_6,
	.reference = ADC_REF_INTERNAL,
	.acquisition_time = ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 40),
	.channel_id = ADC_CHANNEL_ID,
	.input_positive = NRF_SAADC_VDD,
};

static struct adc_sequence sequence = {
	.channels = BIT(ADC_CHANNEL_ID),
	.buffer = &adc_buffer,
	.buffer_size = sizeof(adc_buffer),
	.resolution = ADC_RESOLUTION_BITS,
};
#endif /* BATTERY_ADC_SUPPORTED */

/* Initialize ADC for battery measurement */
static int init_battery_adc(void)
{
#if BATTERY_ADC_SUPPORTED
	adc_dev = DEVICE_DT_GET(ADC_NODE);
	if (!device_is_ready(adc_dev)) {
		LOG_ERR("ADC device not ready");
		return -ENODEV;
	}

	int err = adc_channel_setup(adc_dev, &channel_cfg);
	if (err) {
		LOG_ERR("ADC channel setup failed: %d", err);
		return err;
	}

	adc_available = true;
	LOG_INF("Battery ADC initialized");
	return 0;
#else
	LOG_INF("Battery ADC not supported on this platform (nRF52832 lacks VDD input)");
	return -ENOTSUP;
#endif
}

/* Read battery voltage in millivolts */
static uint16_t read_battery_voltage(void)
{
#if BATTERY_ADC_SUPPORTED
	if (!adc_available) {
		return 0;
	}

	int err = adc_read(adc_dev, &sequence);
	if (err) {
		LOG_ERR("ADC read failed: %d", err);
		return 0;
	}

	/*
	 * Convert ADC reading to millivolts
	 * - Internal reference: 0.6V
	 * - Gain: 1/6 (so full scale = 3.6V)
	 * - 12-bit resolution: 4096 steps
	 * - VDD measurement
	 * Formula: voltage_mv = (adc_value * 3600) / 4096
	 */
	int32_t voltage_mv = (adc_buffer * ADC_REF_MV) / ADC_RESOLUTION;

	return (uint16_t)voltage_mv;
#else
	return 0;
#endif
}

/* Calculate battery percentage from voltage (0% at LOW_THRESHOLD, 100% at FULL) */
static uint8_t voltage_to_percentage(uint16_t voltage_mv)
{
	if (voltage_mv <= BATTERY_LOW_THRESHOLD_MV) {
		return 0;
	}
	if (voltage_mv >= BATTERY_FULL_MV) {
		return 100;
	}
	/* Linear interpolation between LOW_THRESHOLD and FULL */
	uint32_t range = BATTERY_FULL_MV - BATTERY_LOW_THRESHOLD_MV;
	uint32_t offset = voltage_mv - BATTERY_LOW_THRESHOLD_MV;
	return (uint8_t)((offset * 100) / range);
}

/* Update battery voltage and advertising data */
static void update_battery_status(void)
{
	battery.voltage_mv = read_battery_voltage();
	battery.percentage = voltage_to_percentage(battery.voltage_mv);
	battery.is_low = (battery.voltage_mv <= BATTERY_LOW_THRESHOLD_MV);

	/* Update manufacturer data with battery info */
	mfg_data[2] = battery.percentage;                  /* Battery percentage 0-100 */
	mfg_data[3] = battery.is_low ? 0x01 : 0x00;        /* Low battery flag */

	LOG_INF("[BATT] Voltage: %d mV, Percentage: %d%% %s", battery.voltage_mv,
	        battery.percentage, battery.is_low ? "(LOW!)" : "(OK)");

	/* If advertising, update the advertising data */
	if (ble.is_advertising) {
		int err = bt_le_adv_update_data(ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
		if (err) {
			LOG_ERR("Failed to update advertising data: %d", err);
		}
	}
}

/*
 * Update running variance using Welford's online algorithm.
 * This computes variance incrementally in O(1) per sample instead of O(n).
 * Reference: Welford, B.P. (1962). "Note on a method for calculating
 * corrected sums of squares and products".
 */
static void update_variance(float new_value)
{
	motion.count++;
	if (motion.count > VARIANCE_WINDOW_SIZE) {
		/* Approximate sliding window by slowly decaying old values */
		motion.count = VARIANCE_WINDOW_SIZE;
		motion.m2 *= 0.9f;  /* Decay factor to forget old samples */
	}

	float delta = new_value - motion.mean;
	motion.mean += delta / motion.count;
	float delta2 = new_value - motion.mean;
	motion.m2 += delta * delta2;
}

/* Get current variance from Welford's running statistics */
static inline float get_variance(void)
{
	if (motion.count < 2) {
		return 0.0f;
	}
	return motion.m2 / motion.count;
}

/* Fetch acceleration data from sensor */
static int fetch_acceleration(float *x, float *y, float *z)
{
	struct sensor_value accel[3];
	int ret;

	ret = sensor_sample_fetch(lis3dh_dev);
	if (ret < 0) {
		LOG_ERR("Failed to fetch sensor data: %d", ret);
		return ret;
	}

	ret = sensor_channel_get(lis3dh_dev, SENSOR_CHAN_ACCEL_XYZ, accel);
	if (ret < 0) {
		LOG_ERR("Failed to get accel data: %d", ret);
		return ret;
	}

	*x = sensor_value_to_float(&accel[0]);
	*y = sensor_value_to_float(&accel[1]);
	*z = sensor_value_to_float(&accel[2]);

	return 0;
}

/*
 * Calculate squared magnitude of 3D vector.
 * Using squared magnitude avoids expensive sqrt() operation.
 * For comparison purposes, we compare against squared thresholds.
 */
static inline float calculate_magnitude_sq(float x, float y, float z)
{
	return x * x + y * y + z * z;
}

/*
 * Apply EMA filter to smooth magnitude readings.
 * EMA reduces sensor noise while maintaining responsiveness.
 * Formula: filtered = alpha * new + (1 - alpha) * old
 */
static float apply_ema_filter(float new_mag_sq)
{
	if (!motion.ema_initialized) {
		motion.ema_mag_sq = new_mag_sq;
		motion.ema_initialized = true;
		return new_mag_sq;
	}
	motion.ema_mag_sq = EMA_ALPHA * new_mag_sq + (1.0f - EMA_ALPHA) * motion.ema_mag_sq;
	return motion.ema_mag_sq;
}

/*
 * Get absolute delta between current and previous filtered magnitude.
 * Uses EMA filtering to smooth sensor noise, then absolute delta with deadband.
 * Also updates variance using the filtered value (not raw) for consistency.
 * Returns 0 if delta is below noise floor (deadband filtering).
 */
static float get_magnitude_delta(float curr_mag_sq)
{
	/* Apply EMA filter to smooth out sensor quantization noise */
	float filtered_mag = apply_ema_filter(curr_mag_sq);
	
	/* Update variance using FILTERED magnitude (same data source as delta) */
	update_variance(filtered_mag);
	
	/* Calculate absolute delta */
	float diff = filtered_mag - motion.prev_mag_sq;
	float abs_delta = (diff >= 0) ? diff : -diff;
	
	/* Update previous value with filtered magnitude */
	motion.prev_mag_sq = filtered_mag;
	
	/* Apply deadband to filter out noise below threshold */
	if (abs_delta < NOISE_DEADBAND) {
		return 0.0f;
	}
	
	return abs_delta;
}

/* Check for variance-based motion (highway cruising vibrations) */
static bool check_variance_motion(void)
{
	if (motion.count < VARIANCE_WINDOW_SIZE) {
		return false;
	}
	return (get_variance() > VARIANCE_THRESHOLD);
}

/* Apply debounce filter to motion detection */
static bool apply_debounce(bool motion_detected, float delta, float variance,
			   bool delta_trig, bool var_trig)
{
	if (motion_detected) {
		motion.hit_count++;
		if (motion.hit_count >= MIN_MOTION_HITS) {
			LOG_INF("Motion! d=%.2f v=%.2f [%s%s]",
			        (double)delta, (double)variance,
			        delta_trig ? "D" : "", var_trig ? "V" : "");
			motion.hit_count = 0;  /* Reset after confirmed */
			return true;
		}
	} else {
		motion.hit_count = 0;  /* Reset counter if no motion detected */
	}
	return false;
}

/*
 * Detect motion using dual algorithm:
 * 1. Sudden change detection - catches starts, stops, turns (EMA filtered + deadband)
 * 2. Variance detection - catches highway cruising (road vibrations)
 *
 * Uses EMA filtering to smooth sensor noise and deadband to ignore quantization artifacts.
 */
static bool detect_motion(void)
{
	float x, y, z;
	if (fetch_acceleration(&x, &y, &z) < 0) {
		return false;
	}

	float mag_sq = calculate_magnitude_sq(x, y, z);
	
	/* get_magnitude_delta applies EMA filter and also updates variance internally */
	float delta = get_magnitude_delta(mag_sq);
	float variance = get_variance();
	bool delta_triggered = (delta > MOTION_THRESHOLD);
	bool variance_triggered = check_variance_motion();
	bool motion_detected = delta_triggered || variance_triggered;

#ifdef CONFIG_BEACON_CALIBRATION
	/*
	 * CALIBRATION MODE: Log all values to help tune thresholds.
	 * Enable with CONFIG_BEACON_CALIBRATION=y in prj.conf
	 *
	 * How to calibrate:
	 * 1. Enable calibration mode
	 * 2. Place device at rest - delta should be 0 (noise filtered by deadband)
	 * 3. Simulate your motion patterns - note the delta values
	 * 4. Set MOTION_THRESHOLD between noise floor and motion values
	 * 5. Adjust NOISE_DEADBAND if too many/few readings pass through
	 */
	LOG_INF("[CAL] d=%.2f v=%.2f mag=%.1f ema=%.1f | D:%d V:%d",
		(double)delta, (double)variance, (double)mag_sq, (double)motion.ema_mag_sq,
		delta_triggered, variance_triggered);
#endif

	return apply_debounce(motion_detected, delta, variance,
			      delta_triggered, variance_triggered);
}

/* Start BLE advertising */
static int start_advertising(void)
{
	if (ble.is_advertising) {
		return 0; /* Already advertising */
	}

	int err = bt_le_adv_start(&adv_param, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (err) {
		LOG_ERR("Failed to start advertising: %d", err);
		return err;
	}

	ble.is_advertising = true;
	ble.last_adv_log_time = k_uptime_get();  /* Reset log timer */
	set_led(1); /* LED ON when advertising */
	LOG_INF(">>> BLE advertising STARTED (motion detected)");
	LOG_INF("[ADV] Device name: %s, timeout: %d sec", CONFIG_BT_DEVICE_NAME, NO_MOTION_TIMEOUT_SEC);
	return 0;
}

/* Stop BLE advertising */
static int stop_advertising(void)
{
	if (!ble.is_advertising) {
		return 0; /* Already stopped */
	}

	int err = bt_le_adv_stop();
	if (err) {
		LOG_ERR("Failed to stop advertising: %d", err);
		return err;
	}

	ble.is_advertising = false;
	set_led(0); /* LED OFF when sleeping */
	LOG_INF("<<< BLE advertising STOPPED (no motion for %d seconds)", NO_MOTION_TIMEOUT_SEC);

	/* Reset motion detection state for fresh start */
	/* Note: Keep prev_mag_sq and EMA state to avoid false trigger on first sample */
	motion.hit_count = 0;
	motion.count = 0;
	motion.m2 = 0.0f;
	motion.mean = 0.0f;
	/* motion.prev_mag_sq, ema_mag_sq, ema_initialized intentionally NOT reset */

	return 0;
}

/* BLE ready callback */
static void bt_ready(int err)
{
	if (err) {
		LOG_ERR("Bluetooth init failed: %d", err);
		return;
	}

	LOG_INF("Bluetooth initialized successfully");
	k_sem_give(&bt_ready_sem);
}

/* ========== Helper Functions ========== */

/* Calculate elapsed time in seconds */
static inline int64_t elapsed_seconds_since(int64_t now, int64_t since)
{
	return (now - since) / 1000;
}

/* ========== Direct Hardware Interrupt Configuration ========== */

/* Counter to track interrupt frequency */
static volatile uint32_t int1_count;

/* INT1 GPIO callback - called from interrupt context */
static void int1_callback(const struct device *dev, struct gpio_callback *cb,
			  uint32_t pins)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);
	int1_count++;
	k_sem_give(&data_ready_sem);
}

/*
 * Configure LIS2DH registers directly via I2C.
 * This bypasses the Zephyr sensor trigger API which doesn't work properly.
 */
static int configure_lis2dh_interrupt(void)
{
	const struct device *i2c_dev = DEVICE_DT_GET(DT_BUS(DT_NODELABEL(lis3dh)));
	uint8_t reg_val;
	int ret;

	if (!device_is_ready(i2c_dev)) {
		LOG_ERR("I2C device not ready for direct register access");
		return -ENODEV;
	}

	/* First, check and configure CTRL_REG1 to ensure sensor is outputting data */
	ret = i2c_reg_read_byte(i2c_dev, LIS3DH_I2C_ADDR, LIS2DH_REG_CTRL_REG1, &reg_val);
	if (ret < 0) {
		LOG_ERR("Failed to read CTRL_REG1: %d", ret);
		return ret;
	}
	LOG_INF("CTRL_REG1 before: 0x%02X (ODR=%d)", reg_val, (reg_val >> 4) & 0x0F);

	/* Configure: 1Hz ODR, low-power mode, all axes enabled (lowest power) */
	reg_val = LIS2DH_ODR_1HZ | LIS2DH_LPEN | LIS2DH_XYZ_EN;
	ret = i2c_reg_write_byte(i2c_dev, LIS3DH_I2C_ADDR, LIS2DH_REG_CTRL_REG1, reg_val);
	if (ret < 0) {
		LOG_ERR("Failed to write CTRL_REG1: %d", ret);
		return ret;
	}

	/* Verify CTRL_REG1 */
	ret = i2c_reg_read_byte(i2c_dev, LIS3DH_I2C_ADDR, LIS2DH_REG_CTRL_REG1, &reg_val);
	if (ret < 0) {
		LOG_ERR("Failed to verify CTRL_REG1: %d", ret);
		return ret;
	}
	LOG_INF("CTRL_REG1 after: 0x%02X (ODR=%d, 1Hz)", reg_val, (reg_val >> 4) & 0x0F);

	/* Now configure CTRL_REG3 for data-ready interrupt on INT1 */
	ret = i2c_reg_read_byte(i2c_dev, LIS3DH_I2C_ADDR, LIS2DH_REG_CTRL_REG3, &reg_val);
	if (ret < 0) {
		LOG_ERR("Failed to read CTRL_REG3: %d", ret);
		return ret;
	}
	LOG_INF("CTRL_REG3 before: 0x%02X", reg_val);

	/* Enable data-ready interrupt on INT1 */
	reg_val |= LIS2DH_I1_DRDY1;
	ret = i2c_reg_write_byte(i2c_dev, LIS3DH_I2C_ADDR, LIS2DH_REG_CTRL_REG3, reg_val);
	if (ret < 0) {
		LOG_ERR("Failed to write CTRL_REG3: %d", ret);
		return ret;
	}

	/* Verify the write */
	ret = i2c_reg_read_byte(i2c_dev, LIS3DH_I2C_ADDR, LIS2DH_REG_CTRL_REG3, &reg_val);
	if (ret < 0) {
		LOG_ERR("Failed to verify CTRL_REG3: %d", ret);
		return ret;
	}
	LOG_INF("CTRL_REG3 after: 0x%02X (I1_DRDY1=%s)", reg_val,
		(reg_val & LIS2DH_I1_DRDY1) ? "ON" : "OFF");

	if (!(reg_val & LIS2DH_I1_DRDY1)) {
		LOG_ERR("Failed to enable I1_DRDY1 bit!");
		return -EIO;
	}

	/* Read STATUS_REG to verify sensor is producing data */
	ret = i2c_reg_read_byte(i2c_dev, LIS3DH_I2C_ADDR, LIS2DH_REG_STATUS_REG, &reg_val);
	if (ret < 0) {
		LOG_WRN("Failed to read STATUS_REG: %d", ret);
	} else {
		LOG_INF("STATUS_REG: 0x%02X (ZYXDA=%d)", reg_val, (reg_val >> 3) & 1);
	}

	return 0;
}

/*
 * Setup GPIO interrupt on INT1 pin.
 * This catches the hardware interrupt directly.
 */
static int setup_int1_gpio_interrupt(void)
{
	int ret;

	if (!gpio_is_ready_dt(&int1_gpio)) {
		LOG_ERR("INT1 GPIO device not ready");
		return -ENODEV;
	}

	/* Configure as input with interrupt on rising edge */
	ret = gpio_pin_configure_dt(&int1_gpio, GPIO_INPUT);
	if (ret < 0) {
		LOG_ERR("Failed to configure INT1 pin: %d", ret);
		return ret;
	}

	ret = gpio_pin_interrupt_configure_dt(&int1_gpio, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret < 0) {
		LOG_ERR("Failed to configure INT1 interrupt: %d", ret);
		return ret;
	}

	/* Setup callback */
	gpio_init_callback(&int1_cb_data, int1_callback, BIT(int1_gpio.pin));
	ret = gpio_add_callback(int1_gpio.port, &int1_cb_data);
	if (ret < 0) {
		LOG_ERR("Failed to add INT1 callback: %d", ret);
		return ret;
	}

	/* Check current INT1 state */
	int pin_state = gpio_pin_get_dt(&int1_gpio);
	LOG_INF("INT1 GPIO configured on pin %d, current state: %d", int1_gpio.pin, pin_state);

	/*
	 * If INT1 is already HIGH, we need to clear it by reading the data.
	 * Otherwise we'll miss the first rising edge since we're edge-triggered.
	 */
	if (pin_state == 1) {
		LOG_INF("INT1 already high - clearing by reading sensor data...");
		/* Reading data clears the DRDY flag and lowers INT1 */
		int err = sensor_sample_fetch(lis3dh_dev);
		if (err) {
			LOG_WRN("Failed to clear pending data: %d", err);
		}
		k_msleep(10); /* Small delay for INT1 to settle */
		pin_state = gpio_pin_get_dt(&int1_gpio);
		LOG_INF("INT1 state after clearing: %d", pin_state);
	}

	return 0;
}

/*
 * Setup complete hardware interrupt chain:
 * 1. Configure LIS2DH to generate DRDY on INT1
 * 2. Configure nRF52 GPIO to catch INT1 rising edge
 */
static int setup_hardware_interrupt(void)
{
	int ret;

	/* Check if INT1 GPIO is available in device tree */
	if (!int1_gpio.port) {
		LOG_WRN("INT1 GPIO not configured in device tree");
		return -ENODEV;
	}

	/* Configure LIS2DH registers directly */
	ret = configure_lis2dh_interrupt();
	if (ret < 0) {
		return ret;
	}

	/* Setup GPIO interrupt */
	ret = setup_int1_gpio_interrupt();
	if (ret < 0) {
		return ret;
	}

	LOG_INF("Hardware interrupt chain configured successfully");
	return 0;
}

/* Print startup banner with configuration */
static void print_startup_banner(void)
{
	LOG_INF("==========================================");
	LOG_INF("  Motion-Triggered BLE Beacon");
	LOG_INF("==========================================");
	LOG_INF("Motion threshold: %.2f", (double)MOTION_THRESHOLD);
	LOG_INF("Noise deadband: %.2f", (double)NOISE_DEADBAND);
	LOG_INF("EMA alpha: %.2f", (double)EMA_ALPHA);
	LOG_INF("Variance threshold: %.2f", (double)VARIANCE_THRESHOLD);
	LOG_INF("Min motion hits: %d (debounce)", MIN_MOTION_HITS);
	LOG_INF("Sample interval: %d ms", ACCEL_SAMPLE_INTERVAL_MS);
	LOG_INF("No-motion timeout: %d seconds", NO_MOTION_TIMEOUT_SEC);
	LOG_INF("Battery update interval: %d seconds", BATTERY_UPDATE_INTERVAL_SEC);
	LOG_INF("Battery range: %d mV (0%%) - %d mV (100%%)", BATTERY_LOW_THRESHOLD_MV, BATTERY_FULL_MV);
	LOG_INF("Device name: %s", CONFIG_BT_DEVICE_NAME);
	LOG_INF("==========================================");
}

/* Initialize all peripherals (LED, accelerometer, battery ADC) */
static int init_peripherals(void)
{
	/* Initialize LED */
	if (init_led() < 0) {
		LOG_WRN("LED init failed - continuing without LED feedback");
	}

	/* Startup blink */
	for (int i = 0; i < STARTUP_BLINK_COUNT; i++) {
		set_led(1);
		k_msleep(STARTUP_BLINK_MS);
		set_led(0);
		k_msleep(STARTUP_BLINK_MS);
	}

	/* Initialize LIS3DH accelerometer */
	LOG_INF("Initializing LIS3DH accelerometer...");
	int err = init_lis3dh();
	if (err) {
		LOG_ERR("LIS3DH init failed: %d - cannot detect motion!", err);
		return err;
	}

#ifdef CONFIG_BEACON_DIAGNOSTICS
	/* Run diagnostic check to verify sensor is alive (optional) */
	err = check_lis3dh_alive();
	if (err) {
		LOG_ERR("LIS3DH diagnostic failed: %d", err);
		return err;
	}
#endif

	/* Initialize battery ADC */
	LOG_INF("Initializing battery monitor...");
	err = init_battery_adc();
	if (err) {
		LOG_WRN("Battery ADC init failed: %d - continuing without battery monitoring", err);
	} else {
		/* Initial battery reading */
		update_battery_status();
	}

	/* Setup hardware interrupt for low-power data-ready wake */
	LOG_INF("Configuring hardware interrupt...");
	err = setup_hardware_interrupt();
	if (err) {
		LOG_WRN("Hardware interrupt setup failed: %d - falling back to polling", err);
		hw_interrupt_available = false;
	} else {
		hw_interrupt_available = true;
		LOG_INF("Hardware interrupt enabled (true low-power mode)");
	}

	return 0;
}

/* Initialize Bluetooth and wait for it to be ready */
static int init_bluetooth(void)
{
	LOG_INF("Enabling Bluetooth...");
	int err = bt_enable(bt_ready);
	if (err) {
		LOG_ERR("Bluetooth enable failed: %d", err);
		return err;
	}

	/* Wait for BT to be ready */
	err = k_sem_take(&bt_ready_sem, K_SECONDS(5));
	if (err) {
		LOG_ERR("Bluetooth ready timeout!");
		return err;
	}

	/* Explicitly ensure advertising is stopped (clean state) */
	bt_le_adv_stop();
	ble.is_advertising = false;

	return 0;
}

/* Process advertising timeout and status logging */
static void process_advertising_state(int64_t now)
{
	if (!ble.is_advertising) {
		return;
	}

	int64_t elapsed_sec = elapsed_seconds_since(now, ble.last_motion_time);

	/* Check if we should stop advertising due to no motion */
	if (elapsed_sec >= NO_MOTION_TIMEOUT_SEC) {
		stop_advertising();
		return;
	}

	/* Periodic advertising status log */
	int64_t log_elapsed = elapsed_seconds_since(now, ble.last_adv_log_time);
	if (log_elapsed >= ADV_LOG_INTERVAL_SEC) {
		LOG_INF("[ADV] Active for %lld sec, timeout in %lld sec",
		        elapsed_sec, NO_MOTION_TIMEOUT_SEC - elapsed_sec);
		ble.last_adv_log_time = now;
	}
}

/* Process periodic battery updates */
static void process_battery_update(int64_t now)
{
	int64_t elapsed = elapsed_seconds_since(now, battery.last_update);
	if (elapsed >= BATTERY_UPDATE_INTERVAL_SEC) {
		update_battery_status();
		battery.last_update = now;
	}
}

/* Main application loop */
static void run_main_loop(void)
{
	/* Start in sleep mode - NOT advertising */
	set_led(0);  /* Ensure LED is OFF */

	if (hw_interrupt_available) {
		LOG_INF("Using hardware interrupt (true low-power mode)");
		/* Clear any pending interrupts from initialization */
		k_sem_reset(&data_ready_sem);
	} else {
		LOG_INF("Using polling for motion detection (fallback)");
		LOG_INF("Idle poll interval: %d ms", IDLE_POLL_INTERVAL_MS);
	}
	LOG_INF("(Move the device to start BLE advertising)");
	LOG_INF("BLE is OFF - device should NOT be discoverable");

	/* Initialize EMA filter and prev_mag_sq with first reading to avoid startup spike */
	{
		float x, y, z;
		if (fetch_acceleration(&x, &y, &z) == 0) {
			float mag_sq = calculate_magnitude_sq(x, y, z);
			motion.ema_mag_sq = mag_sq;
			motion.prev_mag_sq = mag_sq;
			motion.ema_initialized = true;
			LOG_INF("Initial magnitude^2: %.1f (EMA initialized)", (double)mag_sq);
		}
	}

	/* Initialize motion time (set to past so we don't immediately timeout) */
	ble.last_motion_time = 0;

	while (1) {
		if (ble.is_advertising) {
			int64_t now = k_uptime_get();

			/*
			 * When advertising, use interrupt if available.
			 * Wait for data-ready with timeout to ensure we process timeouts.
			 */
			if (hw_interrupt_available) {
				k_sem_take(&data_ready_sem, K_MSEC(200));
			}

			if (detect_motion()) {
				ble.last_motion_time = now;
			}
			process_advertising_state(now);
			process_battery_update(now);

			if (!hw_interrupt_available) {
				k_msleep(ACCEL_SAMPLE_INTERVAL_MS);
			}
		} else if (hw_interrupt_available) {
			/*
			 * TRUE LOW POWER MODE:
			 * Wait indefinitely for hardware data-ready interrupt.
			 * nRF52 enters System ON sleep (~1.9uA).
			 * LIS2DH @ 1Hz generates interrupt every 1 second (~2uA).
			 * Total idle: ~10uA average.
			 */
			k_sem_take(&data_ready_sem, K_FOREVER);

			if (detect_motion()) {
				LOG_INF("Motion detected (HW interrupt) - waking up!");
				ble.last_motion_time = k_uptime_get();
				start_advertising();
			}
		} else {
			/*
			 * Polling fallback when interrupt unavailable.
			 * Check for motion every 2 seconds.
			 */
			if (detect_motion()) {
				LOG_INF("Motion detected (polling) - waking up!");
				ble.last_motion_time = k_uptime_get();
				start_advertising();
			}
			k_msleep(IDLE_POLL_INTERVAL_MS);
		}
	}
}

int main(void)
{
	int err;

	print_startup_banner();

	err = init_peripherals();
	if (err) {
		return err;
	}

	err = init_bluetooth();
	if (err) {
		return err;
	}

	run_main_loop();

	return 0;  /* Never reached */
}
