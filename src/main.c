/*
 * Motion-Triggered BLE Beacon for nRF52 with LIS3DH Accelerometer
 *
 * - Starts in sleep mode (no BLE advertising)
 * - Detects motion via accelerometer
 * - Starts BLE advertising when motion detected
 * - Stops advertising after NO_MOTION_TIMEOUT_SEC of no motion
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/logging/log.h>

#ifdef CONFIG_ADC_NRFX_SAADC
#include <hal/nrf_saadc.h>
#endif

LOG_MODULE_REGISTER(ble_beacon, LOG_LEVEL_INF);

/* Motion detection - fully hardware-driven */
#define NO_MOTION_TIMEOUT_SEC     30      /* Stop advertising after no motion for this long */

/* Gravity sanity check bounds for diagnostics (m/s²) - squared for comparison */
#define GRAVITY_MIN_MS2_SQ        25.0f   /* (5.0)^2 */
#define GRAVITY_MAX_MS2_SQ        225.0f  /* (15.0)^2 */

/* ADC conversion constants */
#define ADC_REF_MV                3600    /* Full scale voltage (0.6V * 6 gain) */
#define ADC_RESOLUTION            4096    /* 12-bit resolution */

/* ========== LIS2DH Direct Register Configuration ========== */
/* Register addresses */
#define LIS2DH_REG_CTRL_REG1      0x20
#define LIS2DH_REG_CTRL_REG2      0x21
#define LIS2DH_REG_CTRL_REG3      0x22
#define LIS2DH_REG_CTRL_REG4      0x23
#define LIS2DH_REG_CTRL_REG5      0x24
#define LIS2DH_REG_STATUS_REG     0x27
#define LIS2DH_REG_REFERENCE      0x26

/* CTRL_REG2 bits - High-pass filter configuration */
#define LIS2DH_HP_IA1             BIT(0)  /* HP filter enabled for AOI on INT1 */
#define LIS2DH_HPIS1              BIT(1)  /* HP filter enabled for interrupt 1 source */
#define LIS2DH_FDS                BIT(3)  /* Filtered data selection */
#define LIS2DH_HPCF_MASK          0x30    /* HP filter cutoff frequency */

/* CTRL_REG3 bits - INT1 configuration */
#define LIS2DH_I1_DRDY1           BIT(4)  /* Data-ready interrupt on INT1 */
#define LIS2DH_I1_AOI1            BIT(6)  /* AOI1 interrupt on INT1 */

/* Hardware motion detection registers */
#define LIS2DH_REG_INT1_CFG       0x30    /* INT1 configuration */
#define LIS2DH_REG_INT1_SRC       0x31    /* INT1 source (read to clear) */
#define LIS2DH_REG_INT1_THS       0x32    /* INT1 threshold */
#define LIS2DH_REG_INT1_DURATION  0x33    /* INT1 duration */

/* INT1_CFG bits - motion detection configuration */
#define LIS2DH_INT1_ZHIE          BIT(5)  /* Z high event enable */
#define LIS2DH_INT1_ZLIE          BIT(4)  /* Z low event enable */
#define LIS2DH_INT1_YHIE          BIT(3)  /* Y high event enable */
#define LIS2DH_INT1_YLIE          BIT(2)  /* Y low event enable */
#define LIS2DH_INT1_XHIE          BIT(1)  /* X high event enable */
#define LIS2DH_INT1_XLIE          BIT(0)  /* X low event enable */
#define LIS2DH_INT1_6D            BIT(6)  /* 6-direction detection */
#define LIS2DH_INT1_AOI           BIT(7)  /* AND/OR combination */

/* Hardware motion threshold (16mg per LSB in low-power 2g mode) */
#define HW_MOTION_THRESHOLD_MG    48      /* ~48mg threshold - sensitive to vibrations */
#define HW_MOTION_THS_VALUE       (HW_MOTION_THRESHOLD_MG / 16)  /* Register value (3) */
#define HW_MOTION_DURATION        1       /* Require 1 sample above threshold (minimal debounce) */

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

/* Battery monitoring configuration (2x AA alkaline batteries) */
#define BATTERY_UPDATE_INTERVAL_SEC  300  /* Update battery in advertising every 5 min */
#define BATTERY_LOW_THRESHOLD_MV     2000 /* Low battery warning threshold (mV) - 0% */
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
 * Connectable, scannable advertising for Android CDM compatibility.
 * Advertising interval kept at 250-400ms for reliable discovery.
 */
static struct bt_le_adv_param adv_param = BT_LE_ADV_PARAM_INIT(
	BT_LE_ADV_OPT_CONN | BT_LE_ADV_OPT_SCANNABLE,
	0x0190,  /* 250ms min (400 * 0.625ms) */
	0x0280,  /* 400ms max (640 * 0.625ms) */
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
 * ADC for battery voltage measurement using AIN1 (P0.03).
 * Measures 2x AA alkaline battery voltage directly (no voltage divider needed).
 * Voltage range: 1.8V (dead) to 3.0V (fresh), within ADC's 3.6V max.
 */
#if defined(CONFIG_ADC)
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
	.input_positive = NRF_SAADC_INPUT_AIN1,  /* P0.03 - connected to battery+ */
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
	LOG_INF("Battery ADC initialized (AIN1/P0.03)");
	return 0;
#else
	LOG_INF("Battery ADC not enabled - enable CONFIG_ADC in prj.conf");
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
 * Configure LIS2DH for hardware motion detection (deep sleep mode).
 * INT1 fires only when acceleration exceeds threshold on any axis.
 * MCU can sleep indefinitely until real motion occurs.
 */
static int configure_hardware_motion_mode(void)
{
	const struct device *i2c_dev = DEVICE_DT_GET(DT_BUS(DT_NODELABEL(lis3dh)));
	uint8_t reg_val;
	int ret;

	if (!device_is_ready(i2c_dev)) {
		LOG_ERR("I2C device not ready");
		return -ENODEV;
	}

	LOG_INF("Configuring hardware motion detection mode...");

	/* Configure CTRL_REG1: 1Hz ODR (lowest power), low-power mode, XYZ enabled */
	reg_val = LIS2DH_ODR_1HZ | LIS2DH_LPEN | LIS2DH_XYZ_EN;
	ret = i2c_reg_write_byte(i2c_dev, LIS3DH_I2C_ADDR, LIS2DH_REG_CTRL_REG1, reg_val);
	if (ret < 0) {
		LOG_ERR("Failed to write CTRL_REG1: %d", ret);
		return ret;
	}

	/*
	 * Configure CTRL_REG2: Enable high-pass filter for interrupt 1.
	 * This removes the DC component (gravity) so we only detect CHANGES
	 * in acceleration, not the static 1g from gravity.
	 * HPCF[1:0] = 11 (0x30) = lowest cutoff frequency for slow motion detection
	 */
	reg_val = LIS2DH_HP_IA1 | 0x30;  /* HP filter + lowest cutoff frequency */
	ret = i2c_reg_write_byte(i2c_dev, LIS3DH_I2C_ADDR, LIS2DH_REG_CTRL_REG2, reg_val);
	if (ret < 0) {
		LOG_ERR("Failed to write CTRL_REG2: %d", ret);
		return ret;
	}

	/*
	 * Read REFERENCE register to reset the HP filter.
	 * This sets the current acceleration as the "zero" reference.
	 */
	ret = i2c_reg_read_byte(i2c_dev, LIS3DH_I2C_ADDR, LIS2DH_REG_REFERENCE, &reg_val);
	if (ret < 0) {
		LOG_WRN("Failed to read REFERENCE: %d", ret);
	}

	/* Disable DRDY interrupt, enable AOI1 (motion) interrupt on INT1 */
	reg_val = LIS2DH_I1_AOI1;  /* AOI1 interrupt on INT1 */
	ret = i2c_reg_write_byte(i2c_dev, LIS3DH_I2C_ADDR, LIS2DH_REG_CTRL_REG3, reg_val);
	if (ret < 0) {
		LOG_ERR("Failed to write CTRL_REG3: %d", ret);
		return ret;
	}

	/* Configure INT1_THS - motion threshold */
	ret = i2c_reg_write_byte(i2c_dev, LIS3DH_I2C_ADDR, LIS2DH_REG_INT1_THS, HW_MOTION_THS_VALUE);
	if (ret < 0) {
		LOG_ERR("Failed to write INT1_THS: %d", ret);
		return ret;
	}

	/* Configure INT1_DURATION - minimum duration (0 = immediate) */
	ret = i2c_reg_write_byte(i2c_dev, LIS3DH_I2C_ADDR, LIS2DH_REG_INT1_DURATION, HW_MOTION_DURATION);
	if (ret < 0) {
		LOG_ERR("Failed to write INT1_DURATION: %d", ret);
		return ret;
	}

	/* Configure INT1_CFG - OR combination of high events on X, Y, Z (any axis movement) */
	reg_val = LIS2DH_INT1_XHIE | LIS2DH_INT1_YHIE | LIS2DH_INT1_ZHIE;  /* High event on any axis */
	ret = i2c_reg_write_byte(i2c_dev, LIS3DH_I2C_ADDR, LIS2DH_REG_INT1_CFG, reg_val);
	if (ret < 0) {
		LOG_ERR("Failed to write INT1_CFG: %d", ret);
		return ret;
	}

	/* Read INT1_SRC to clear any pending interrupt */
	ret = i2c_reg_read_byte(i2c_dev, LIS3DH_I2C_ADDR, LIS2DH_REG_INT1_SRC, &reg_val);
	if (ret < 0) {
		LOG_WRN("Failed to read INT1_SRC: %d", ret);
	}

	/* Wait for interrupt engine to settle and clear again */
	k_msleep(100);
	ret = i2c_reg_read_byte(i2c_dev, LIS3DH_I2C_ADDR, LIS2DH_REG_INT1_SRC, &reg_val);
	if (ret < 0) {
		LOG_WRN("Failed to read INT1_SRC (2nd): %d", ret);
	}

	LOG_INF("Hardware motion mode active (threshold: %dmg, duration: %d samples)",
		HW_MOTION_THRESHOLD_MG, HW_MOTION_DURATION);
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
 * 1. Configure LIS2DH for hardware motion detection (deep sleep)
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

	/* Start in hardware motion detection mode (deep sleep) */
	ret = configure_hardware_motion_mode();
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
	LOG_INF("Mode: Hardware motion detection");
	LOG_INF("Motion threshold: %d mg", HW_MOTION_THRESHOLD_MG);
	LOG_INF("Motion duration: %d samples", HW_MOTION_DURATION);
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
	if (err == -ENOTSUP) {
		LOG_INF("Battery ADC not enabled in build config");
	} else if (err) {
		LOG_WRN("Battery ADC init failed: %d - continuing without battery monitoring", err);
	} else {
		/* Initial battery reading */
		update_battery_status();
	}

	/* Setup hardware interrupt for low-power data-ready wake */
	LOG_INF("Configuring hardware interrupt...");
	err = setup_hardware_interrupt();
	if (err) {
		LOG_ERR("Hardware interrupt setup failed: %d - cannot operate without interrupt!", err);
		return err;
	}
	LOG_INF("Hardware interrupt enabled (interrupt-only mode)");

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
#if BATTERY_ADC_SUPPORTED
	int64_t elapsed = elapsed_seconds_since(now, battery.last_update);
	if (elapsed >= BATTERY_UPDATE_INTERVAL_SEC) {
		update_battery_status();
		battery.last_update = now;
	}
#else
	ARG_UNUSED(now);
#endif
}

/*
 * Main application loop - fully hardware-driven motion detection.
 *
 * Simple state machine:
 * - Sleep: Wait for hardware motion interrupt (~1s latency at 1Hz ODR)
 * - Active: BLE advertising, reset timeout on each motion interrupt
 * - Timeout: No motion for 30s -> back to sleep
 */
static void run_main_loop(void)
{
	/* Start in sleep mode - NOT advertising */
	set_led(0);

	LOG_INF("Hardware motion detection mode:");
	LOG_INF("  - Motion interrupt -> start advertising");
	LOG_INF("  - No motion for %d sec -> stop advertising", NO_MOTION_TIMEOUT_SEC);
	k_sem_reset(&data_ready_sem);

	ble.last_motion_time = 0;

	while (1) {
		if (ble.is_advertising) {
			/*
			 * ACTIVE MODE:
			 * Wait for motion interrupt with timeout.
			 * Each interrupt resets the no-motion timer.
			 */
			int sem_ret = k_sem_take(&data_ready_sem, K_MSEC(500));
			int64_t now = k_uptime_get();

			if (sem_ret == 0) {
				/* Motion detected - reset timeout */
				ble.last_motion_time = now;
			}

			process_advertising_state(now);
			process_battery_update(now);
		} else {
			/*
			 * SLEEP MODE:
			 * Wait indefinitely for hardware motion interrupt.
			 * nRF52 enters System ON sleep (~1.9uA).
			 * LIS2DH monitors motion, only wakes MCU when threshold exceeded.
			 */
			LOG_INF("Entering sleep (waiting for motion)...");
			k_sem_reset(&data_ready_sem);
			k_sem_take(&data_ready_sem, K_FOREVER);

			/* Motion detected - start advertising */
			LOG_INF("Motion detected - waking up!");
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
