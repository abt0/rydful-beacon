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
/* Using squared thresholds to avoid expensive sqrt operations */
#define MOTION_THRESHOLD_MS2_SQ   0.0625f /* (0.25)^2 - Threshold for sudden changes */
#define VARIANCE_THRESHOLD        0.01f   /* Variance threshold for detecting road vibrations */
#define MIN_MOTION_HITS           3       /* Require multiple hits to confirm motion */
#define VARIANCE_WINDOW_SIZE      16      /* Number of samples for variance calculation */
#define NO_MOTION_TIMEOUT_SEC     60      /* Timeout for highway cruising gaps */
#define ACCEL_SAMPLE_INTERVAL_MS  100     /* Sampling interval when advertising (ms) */
#define MOTION_CHECK_INTERVAL_MS  1000    /* Interval to check for motion timeout when idle (ms) */

/* LED feedback configuration */
#define STARTUP_BLINK_COUNT       3       /* Number of blinks on startup */
#define STARTUP_BLINK_MS          100     /* Duration of each startup blink (ms) */

/* Gravity sanity check bounds (m/s²) - squared for comparison */
#define GRAVITY_MIN_MS2_SQ        25.0f   /* (5.0)^2 */
#define GRAVITY_MAX_MS2_SQ        225.0f  /* (15.0)^2 */

/* ADC conversion constants */
#define ADC_REF_MV                3600    /* Full scale voltage (0.6V * 6 gain) */
#define ADC_RESOLUTION            4096    /* 12-bit resolution */

/* LIS3DH accelerometer device */
static const struct device *lis3dh_dev;

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
	float prev_mag_sq;    /* Previous magnitude squared */
	int hit_count;        /* Debounce counter */
};
static struct motion_state motion = {
	.mean = 0.0f,
	.m2 = 0.0f,
	.count = 0,
	.prev_mag_sq = 0.0f,
	.hit_count = 0,
};

/* Semaphore to wait for BT ready */
static K_SEM_DEFINE(bt_ready_sem, 0, 1);

/* Semaphore to signal motion detected via interrupt */
static K_SEM_DEFINE(motion_sem, 0, 1);

/* Forward declaration for motion trigger setup */
static int setup_motion_trigger(void);

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

/* ADC device and channel configuration for nRF52 VDD measurement */
#if DT_NODE_EXISTS(DT_PATH(zephyr_user)) && DT_NODE_HAS_PROP(DT_PATH(zephyr_user), io_channels)
#define ADC_NODE DT_PHANDLE(DT_PATH(zephyr_user), io_channels)
#else
#define ADC_NODE DT_NODELABEL(adc)
#endif

static const struct device *adc_dev;
static int16_t adc_buffer;

static struct adc_channel_cfg channel_cfg = {
	.gain = ADC_GAIN_1_6,
	.reference = ADC_REF_INTERNAL,
	.acquisition_time = ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 40),
	.channel_id = 0,
#if defined(CONFIG_ADC_NRFX_SAADC)
	.input_positive = SAADC_CH_PSELP_PSELP_VDD,
#endif
};

static struct adc_sequence sequence = {
	.channels = BIT(0),
	.buffer = &adc_buffer,
	.buffer_size = sizeof(adc_buffer),
	.resolution = 12,
};

/* Initialize ADC for battery measurement */
static int init_battery_adc(void)
{
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

	LOG_INF("Battery ADC initialized");
	return 0;
}

/* Read battery voltage in millivolts */
static uint16_t read_battery_voltage(void)
{
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
 * Get squared delta between current and previous magnitude.
 * Returns (curr - prev)^2 to avoid sqrt and fabs.
 */
static float get_magnitude_delta_sq(float curr_mag_sq)
{
	float diff = curr_mag_sq - motion.prev_mag_sq;
	motion.prev_mag_sq = curr_mag_sq;
	return diff * diff;  /* Squared delta */
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
static bool apply_debounce(bool motion_detected, float delta_sq)
{
	if (motion_detected) {
		motion.hit_count++;
		if (motion.hit_count >= MIN_MOTION_HITS) {
			LOG_INF("Motion confirmed! Delta^2: %.4f (hits: %d)",
			        (double)delta_sq, motion.hit_count);
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
 * 1. Sudden change detection - catches starts, stops, turns
 * 2. Variance detection - catches highway cruising (road vibrations)
 *
 * All comparisons use squared values to avoid sqrt() operations.
 */
static bool detect_motion(void)
{
	float x, y, z;
	if (fetch_acceleration(&x, &y, &z) < 0) {
		return false;
	}

	float mag_sq = calculate_magnitude_sq(x, y, z);
	update_variance(mag_sq);

	/* Check both motion detection algorithms using squared thresholds */
	float delta_sq = get_magnitude_delta_sq(mag_sq);
	bool motion_detected = (delta_sq > MOTION_THRESHOLD_MS2_SQ) || check_variance_motion();

	return apply_debounce(motion_detected, delta_sq);
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

/* Print startup banner with configuration */
static void print_startup_banner(void)
{
	LOG_INF("==========================================");
	LOG_INF("  Motion-Triggered BLE Beacon");
	LOG_INF("==========================================");
	LOG_INF("Sudden change threshold^2: %.4f", (double)MOTION_THRESHOLD_MS2_SQ);
	LOG_INF("Variance threshold: %.4f", (double)VARIANCE_THRESHOLD);
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

	/* Setup motion trigger for interrupt-driven wake */
	err = setup_motion_trigger();
	if (err) {
		LOG_WRN("Motion trigger setup failed: %d - falling back to polling", err);
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

/* Accelerometer trigger callback - called from interrupt context */
static void accel_trigger_handler(const struct device *dev,
				   const struct sensor_trigger *trig)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(trig);
	/* Signal main thread that motion was detected */
	k_sem_give(&motion_sem);
}

/* Setup accelerometer motion trigger with hardware threshold */
static int setup_motion_trigger(void)
{
	int err;

	/*
	 * Configure the motion detection threshold on the LIS3DH.
	 * SENSOR_ATTR_SLOPE_TH sets the threshold for the high-pass filtered
	 * acceleration data. When any axis exceeds this threshold, INT1 fires.
	 * Value is in m/s² - we use a low threshold to detect car engine vibrations.
	 */
	struct sensor_value threshold = {
		.val1 = 0,       /* Integer part: 0 m/s² */
		.val2 = 500000,  /* Fractional part: 0.5 m/s² threshold */
	};

	err = sensor_attr_set(lis3dh_dev, SENSOR_CHAN_ACCEL_XYZ,
			      SENSOR_ATTR_SLOPE_TH, &threshold);
	if (err) {
		LOG_WRN("Failed to set motion threshold: %d (using driver default)", err);
		/* Continue anyway - driver may have a reasonable default */
	} else {
		LOG_INF("Motion threshold set to 0.5 m/s²");
	}

	/*
	 * Configure the trigger to fire on acceleration delta (motion).
	 * The LIS3DH driver maps this to INT1 with high-pass filter enabled.
	 */
	struct sensor_trigger trig = {
		.type = SENSOR_TRIG_DELTA,
		.chan = SENSOR_CHAN_ACCEL_XYZ,
	};

	err = sensor_trigger_set(lis3dh_dev, &trig, accel_trigger_handler);
	if (err) {
		LOG_ERR("Failed to set motion trigger: %d", err);
		return err;
	}

	LOG_INF("Motion trigger configured (interrupt-driven via INT1)");
	return 0;
}

/* Main application loop - interrupt-driven for low power */
static void run_main_loop(void)
{
	/* Start in sleep mode - NOT advertising */
	set_led(0);  /* Ensure LED is OFF */
	LOG_INF("Entering low-power mode - waiting for motion interrupt...");
	LOG_INF("(Move the device to start BLE advertising)");
	LOG_INF("BLE is OFF - device should NOT be discoverable");

	/* Initialize motion time (set to past so we don't immediately timeout) */
	ble.last_motion_time = 0;

	while (1) {
		if (ble.is_advertising) {
			int64_t now = k_uptime_get();

			/* When advertising, poll for continued motion and variance */
			if (detect_motion()) {
				ble.last_motion_time = now;
			}
			process_advertising_state(now);
			process_battery_update(now);
			k_msleep(ACCEL_SAMPLE_INTERVAL_MS);
		} else {
			/*
			 * When idle, wait indefinitely for motion interrupt.
			 * CPU enters deep sleep here - lowest power consumption.
			 * No periodic wake-ups, no battery polling when parked.
			 */
			k_sem_take(&motion_sem, K_FOREVER);

			/* Motion interrupt received - wake up and start advertising */
			LOG_INF("Motion interrupt received - waking up!");
			ble.last_motion_time = k_uptime_get();
			start_advertising();
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
