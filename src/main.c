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

#include <math.h>
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
#define MOTION_THRESHOLD_MS2      0.25f   /* Threshold for sudden changes - 3x above noise floor */
#define VARIANCE_THRESHOLD        0.01f   /* Variance threshold for detecting road vibrations */
#define MIN_MOTION_HITS           3       /* Require multiple hits to confirm motion */
#define VARIANCE_WINDOW_SIZE      16      /* Number of samples for variance calculation */
#define NO_MOTION_TIMEOUT_SEC     60      /* Timeout for highway cruising gaps */
#define ACCEL_SAMPLE_INTERVAL_MS  50      /* Sample faster to capture vibrations (ms) */

/* LED feedback configuration */
#define STARTUP_BLINK_COUNT       3       /* Number of blinks on startup */
#define STARTUP_BLINK_MS          100     /* Duration of each startup blink (ms) */

/* LIS3DH accelerometer device */
static const struct device *lis3dh_dev;

/* LED for visual feedback */
#define LED0_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

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

/* Motion detection state */
struct motion_state {
	float magnitude_history[VARIANCE_WINDOW_SIZE];
	int history_idx;
	int history_count;
	float prev_magnitude;
	int hit_count;  /* Debounce counter */
};
static struct motion_state motion = {
	.magnitude_history = {0},
	.history_idx = 0,
	.history_count = 0,
	.prev_magnitude = 0.0f,
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

/* Connectable, scannable advertising with fast interval */
static struct bt_le_adv_param adv_param = BT_LE_ADV_PARAM_INIT(
	BT_LE_ADV_OPT_CONN | BT_LE_ADV_OPT_SCANNABLE | BT_LE_ADV_OPT_USE_IDENTITY,
	BT_GAP_ADV_FAST_INT_MIN_1,  /* 100ms min */
	BT_GAP_ADV_FAST_INT_MAX_1,  /* 150ms max */
	NULL
);

/* Initialize LED */
static int init_led(void)
{
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
	return 0;
}

/* Get LIS3DH device from device tree node label */
#define LIS3DH_NODE DT_NODELABEL(lis3dh)

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
	int32_t voltage_mv = (adc_buffer * 3600) / 4096;
	
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

/* Calculate variance of magnitude history */
static float calculate_variance(void)
{
	if (motion.history_count < 2) {
		return 0.0f;
	}

	/* Calculate mean */
	float sum = 0.0f;
	for (int i = 0; i < motion.history_count; i++) {
		sum += motion.magnitude_history[i];
	}
	float mean = sum / motion.history_count;

	/* Calculate variance */
	float variance = 0.0f;
	for (int i = 0; i < motion.history_count; i++) {
		float diff = motion.magnitude_history[i] - mean;
		variance += diff * diff;
	}
	variance /= motion.history_count;

	return variance;
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

/* Calculate magnitude of 3D vector */
static inline float calculate_magnitude(float x, float y, float z)
{
	return sqrtf(x * x + y * y + z * z);
}

/* Update magnitude history buffer for variance calculation */
static void update_magnitude_history(float magnitude)
{
	motion.magnitude_history[motion.history_idx] = magnitude;
	motion.history_idx = (motion.history_idx + 1) % VARIANCE_WINDOW_SIZE;
	if (motion.history_count < VARIANCE_WINDOW_SIZE) {
		motion.history_count++;
	}
}

/* Check for sudden change in acceleration (starts, stops, turns) */
static bool check_sudden_change(float curr_magnitude, float *delta_out)
{
	float delta = fabsf(curr_magnitude - motion.prev_magnitude);
	motion.prev_magnitude = curr_magnitude;
	*delta_out = delta;
	return (delta > MOTION_THRESHOLD_MS2);
}

/* Check for variance-based motion (highway cruising vibrations) */
static bool check_variance_motion(void)
{
	if (motion.history_count < VARIANCE_WINDOW_SIZE) {
		return false;
	}
	float variance = calculate_variance();
	return (variance > VARIANCE_THRESHOLD);
}

/* Apply debounce filter to motion detection */
static bool apply_debounce(bool motion_detected, float delta)
{
	if (motion_detected) {
		motion.hit_count++;
		if (motion.hit_count >= MIN_MOTION_HITS) {
			LOG_INF("Motion confirmed! Delta: %.3f m/s² (hits: %d)", 
			        (double)delta, motion.hit_count);
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
 */
static bool detect_motion(void)
{
	float x, y, z;
	if (fetch_acceleration(&x, &y, &z) < 0) {
		return false;
	}

	float magnitude = calculate_magnitude(x, y, z);
	update_magnitude_history(magnitude);

	/* Check both motion detection algorithms */
	float delta;
	bool motion_detected = check_sudden_change(magnitude, &delta);
	if (!motion_detected) {
		motion_detected = check_variance_motion();
	}

	return apply_debounce(motion_detected, delta);
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
	gpio_pin_set_dt(&led, 1); /* LED ON when advertising */
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
	gpio_pin_set_dt(&led, 0); /* LED OFF when sleeping */
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

/* Calculate elapsed time in seconds since a timestamp */
static inline int64_t elapsed_seconds(int64_t since)
{
	return (k_uptime_get() - since) / 1000;
}

/* Print startup banner with configuration */
static void print_startup_banner(void)
{
	LOG_INF("==========================================");
	LOG_INF("  Motion-Triggered BLE Beacon");
	LOG_INF("==========================================");
	LOG_INF("Sudden change threshold: %.2f m/s²", (double)MOTION_THRESHOLD_MS2);
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
		gpio_pin_set_dt(&led, 1);
		k_msleep(STARTUP_BLINK_MS);
		gpio_pin_set_dt(&led, 0);
		k_msleep(STARTUP_BLINK_MS);
	}

	/* Initialize LIS3DH accelerometer */
	LOG_INF("Initializing LIS3DH accelerometer...");
	int err = init_lis3dh();
	if (err) {
		LOG_ERR("LIS3DH init failed: %d - cannot detect motion!", err);
		return err;
	}
	LOG_INF("LIS3DH ready!");

	/* Initialize battery ADC */
	LOG_INF("Initializing battery monitor...");
	err = init_battery_adc();
	if (err) {
		LOG_WRN("Battery ADC init failed: %d - continuing without battery monitoring", err);
	} else {
		/* Initial battery reading */
		update_battery_status();
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

	int64_t elapsed_sec = (now - ble.last_motion_time) / 1000;
	
	/* Check if we should stop advertising due to no motion */
	if (elapsed_sec >= NO_MOTION_TIMEOUT_SEC) {
		stop_advertising();
		return;
	}
	
	/* Periodic advertising status log */
	int64_t log_elapsed = (now - ble.last_adv_log_time) / 1000;
	if (log_elapsed >= ADV_LOG_INTERVAL_SEC) {
		LOG_INF("[ADV] Active for %lld sec, timeout in %lld sec",
		        elapsed_sec, NO_MOTION_TIMEOUT_SEC - elapsed_sec);
		ble.last_adv_log_time = now;
	}
}

/* Process periodic battery updates */
static void process_battery_update(int64_t now)
{
	int64_t elapsed = (now - battery.last_update) / 1000;
	if (elapsed >= BATTERY_UPDATE_INTERVAL_SEC) {
		update_battery_status();
		battery.last_update = now;
	}
}

/* Main application loop */
static void run_main_loop(void)
{
	/* Start in sleep mode - NOT advertising */
	gpio_pin_set_dt(&led, 0);  /* Ensure LED is OFF */
	LOG_INF("Entering sleep mode - waiting for motion...");
	LOG_INF("(Move the device to start BLE advertising)");
	LOG_INF("BLE is OFF - device should NOT be discoverable");

	/* Initialize motion time (set to past so we don't immediately timeout) */
	ble.last_motion_time = 0;

	while (1) {
		int64_t now = k_uptime_get();

		if (detect_motion()) {
			/* Motion detected - update timestamp and start advertising */
			ble.last_motion_time = now;
			start_advertising();
		}

		process_advertising_state(now);
		process_battery_update(now);

		k_msleep(ACCEL_SAMPLE_INTERVAL_MS);
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
