/*
 * BLE Beacon - Hardware Interrupt-Driven Motion Detection
 *
 * Uses LIS3DH accelerometer's internal motion detection engine to wake the MCU
 * only when actual motion occurs, achieving ultra-low power consumption (~2.5 µA sleep).
 *
 * The LIS3DH operates at 1Hz in low-power mode with high-pass filtering to remove
 * gravity, triggering an interrupt only on acceleration *changes* above threshold.
 */

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/logging/log.h>
#include <hal/nrf_saadc.h>

LOG_MODULE_REGISTER(beacon, LOG_LEVEL_INF);

/*
 * Configuration Parameters
 */

/* Motion detection threshold in mg */
#define HW_MOTION_THRESHOLD_MG    32

/* Number of samples above threshold before triggering */
#define HW_MOTION_DURATION        2

/* Seconds of no motion before stopping advertising */
#define NO_MOTION_TIMEOUT_SEC     30

/* Battery monitoring configuration */
#define BATTERY_LOW_THRESHOLD_MV  2000  /* 0% battery level */
#define BATTERY_FULL_MV           3000  /* 100% battery level */
#define BATTERY_UPDATE_INTERVAL_SEC 300 /* Update every 5 minutes */

/* ADC configuration for battery voltage measurement */
#define ADC_NODE                  DT_NODELABEL(adc)
#define ADC_CHANNEL_ID            0
#define ADC_RESOLUTION_BITS       12
#define ADC_GAIN                  ADC_GAIN_1_6
#define ADC_REFERENCE             ADC_REF_INTERNAL
#define ADC_REF_MV                600  /* Internal reference is 0.6V */
#define ADC_RESOLUTION            ((1 << ADC_RESOLUTION_BITS) - 1)  /* 4095 for 12-bit */

/*
 * LIS2DH/LIS3DH Register Definitions
 * (Using direct I2C access for reliable interrupt configuration)
 */

/* Control registers */
#define LIS2DH_REG_CTRL_REG1      0x20
#define LIS2DH_REG_CTRL_REG2      0x21
#define LIS2DH_REG_CTRL_REG3      0x22
#define LIS2DH_REG_CTRL_REG4      0x23
#define LIS2DH_REG_CTRL_REG5      0x24
#define LIS2DH_REG_CTRL_REG6      0x25
#define LIS2DH_REG_REFERENCE      0x26

/* Interrupt configuration registers */
#define LIS2DH_REG_INT1_CFG       0x30
#define LIS2DH_REG_INT1_SRC       0x31
#define LIS2DH_REG_INT1_THS       0x32
#define LIS2DH_REG_INT1_DURATION  0x33

/* Identification */
#define LIS2DH_REG_WHO_AM_I       0x0F
#define LIS2DH_WHO_AM_I_VALUE     0x33

/* CTRL_REG1 bits */
#define LIS2DH_ODR_1HZ            (0x01 << 4)  /* 1Hz output data rate */
#define LIS2DH_LPEN               BIT(3)       /* Low-power mode enable */
#define LIS2DH_XYZ_EN             0x07         /* Enable X, Y, Z axes */

/* CTRL_REG2 bits */
#define LIS2DH_HP_IA1             BIT(0)       /* HP filter for interrupt 1 */
#define LIS2DH_HPCF_LOWEST        (0x03 << 4)  /* Lowest HP cutoff frequency */

/* CTRL_REG3 bits */
#define LIS2DH_I1_AOI1            BIT(6)       /* AOI1 interrupt on INT1 */

/* INT1_CFG bits - motion detection on any axis */
#define LIS2DH_INT1_XHIE          BIT(1)       /* X high event enable */
#define LIS2DH_INT1_YHIE          BIT(3)       /* Y high event enable */
#define LIS2DH_INT1_ZHIE          BIT(5)       /* Z high event enable */

/*
 * Device handles
 */

/* I2C device for direct register access */
static const struct device *i2c_dev;
#define LIS3DH_I2C_ADDR           0x19

/* GPIO for INT1 interrupt - P0.02 */
static const struct gpio_dt_spec int1_gpio = {
	.port = DEVICE_DT_GET(DT_NODELABEL(gpio0)),
	.pin = 2,
	.dt_flags = GPIO_ACTIVE_HIGH
};

/* Semaphore for interrupt signaling */
K_SEM_DEFINE(motion_sem, 0, 1);

/* GPIO callback data */
static struct gpio_callback int1_cb_data;

/*
 * Battery Monitoring
 */

/* ADC device */
static const struct device *adc_dev;
static bool adc_available = false;
static int16_t adc_buffer;

/* Battery state */
static struct {
	uint16_t voltage_mv;
	uint8_t percentage;
	bool is_low;
	int64_t last_update_time;
} battery = {
	.voltage_mv = 3000,
	.percentage = 100,
	.is_low = false,
	.last_update_time = 0
};

/*
 * BLE Advertising Configuration
 */

/* Manufacturer data: [Company ID (2 bytes), Battery %, Low battery flag] */
static uint8_t mfg_data[] = {
	0xFF, 0xFF,  /* Company ID (0xFFFF = test/internal use) */
	100,         /* Battery percentage (0-100) */
	0x00         /* Low battery flag (0x00 = OK, 0x01 = LOW) */
};

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL, 0xAA, 0xFE),
	BT_DATA(BT_DATA_MANUFACTURER_DATA, mfg_data, sizeof(mfg_data)),
};

static const struct bt_data sd[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME,
		sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

/* Connectable, scannable advertising for Android CDM compatibility */
static struct bt_le_adv_param adv_params = BT_LE_ADV_PARAM_INIT(
	BT_LE_ADV_OPT_CONN | BT_LE_ADV_OPT_SCANNABLE,
	0x0190,  /* 250ms min (400 * 0.625ms) */
	0x0280,  /* 400ms max (640 * 0.625ms) */
	NULL
);

/* Application state */
static struct {
	bool is_advertising;
	int64_t last_motion_time;
} app_state;

/*
 * I2C Helper Functions
 */

static int lis3dh_read_reg(uint8_t reg, uint8_t *val)
{
	return i2c_write_read(i2c_dev, LIS3DH_I2C_ADDR, &reg, 1, val, 1);
}

static int lis3dh_write_reg(uint8_t reg, uint8_t val)
{
	uint8_t buf[2] = {reg, val};
	return i2c_write(i2c_dev, buf, 2, LIS3DH_I2C_ADDR);
}

/*
 * Battery Monitoring Functions
 */

/**
 * Initialize ADC for battery voltage measurement
 */
static int init_battery_adc(void)
{
	int err;

	adc_dev = DEVICE_DT_GET(ADC_NODE);
	if (!device_is_ready(adc_dev)) {
		LOG_ERR("ADC device not ready");
		return -ENODEV;
	}

	/* Configure ADC channel */
	struct adc_channel_cfg channel_cfg = {
		.gain = ADC_GAIN,
		.reference = ADC_REFERENCE,
		.acquisition_time = ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 40),
		.channel_id = ADC_CHANNEL_ID,
		.input_positive = NRF_SAADC_INPUT_AIN1,  /* P0.03 */
	};

	err = adc_channel_setup(adc_dev, &channel_cfg);
	if (err) {
		LOG_ERR("ADC channel setup failed: %d", err);
		return err;
	}

	adc_available = true;
	LOG_INF("Battery ADC initialized (AIN1/P0.03, gain=1/6, ref=0.6V)");
	return 0;
}

/**
 * Read battery voltage from ADC
 * Returns voltage in millivolts (mV)
 */
static uint16_t read_battery_voltage(void)
{
	if (!adc_available) {
		return 0;
	}

	int err;
	struct adc_sequence sequence = {
		.channels = BIT(ADC_CHANNEL_ID),
		.buffer = &adc_buffer,
		.buffer_size = sizeof(adc_buffer),
		.resolution = ADC_RESOLUTION_BITS,
	};

	err = adc_read(adc_dev, &sequence);
	if (err) {
		LOG_ERR("ADC read failed: %d", err);
		return 0;
	}

	/* Convert ADC value to millivolts
	 * With 1/6 gain and 0.6V reference:
	 * - ADC measures 0.6V at full scale (4095)
	 * - Input voltage = ADC_value * (0.6V / 4095) * 6 (gain factor)
	 * - In mV: voltage_mv = ADC_value * 600mV * 6 / 4095
	 */
	int32_t voltage_mv = ((int32_t)adc_buffer * ADC_REF_MV * 6) / ADC_RESOLUTION;

	return (uint16_t)voltage_mv;
}

/**
 * Convert battery voltage to percentage (0-100%)
 * Uses linear interpolation between LOW and FULL thresholds
 */
static uint8_t voltage_to_percentage(uint16_t voltage_mv)
{
	if (voltage_mv <= BATTERY_LOW_THRESHOLD_MV) {
		return 0;
	}
	if (voltage_mv >= BATTERY_FULL_MV) {
		return 100;
	}

	uint32_t range = BATTERY_FULL_MV - BATTERY_LOW_THRESHOLD_MV;
	uint32_t offset = voltage_mv - BATTERY_LOW_THRESHOLD_MV;
	return (uint8_t)((offset * 100) / range);
}

/**
 * Update battery status and BLE manufacturer data
 */
static void update_battery_status(void)
{
	battery.voltage_mv = read_battery_voltage();
	battery.percentage = voltage_to_percentage(battery.voltage_mv);
	battery.is_low = (battery.voltage_mv <= BATTERY_LOW_THRESHOLD_MV);
	battery.last_update_time = k_uptime_get();

	/* Update manufacturer data */
	mfg_data[2] = battery.percentage;
	mfg_data[3] = battery.is_low ? 0x01 : 0x00;

	LOG_INF("[BATT] Voltage: %d mV, Percentage: %d%% %s",
		battery.voltage_mv, battery.percentage,
		battery.is_low ? "(LOW!)" : "(OK)");

	/* Update advertising data if currently advertising */
	if (app_state.is_advertising) {
		int err = bt_le_adv_update_data(ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
		if (err) {
			LOG_ERR("Failed to update advertising data: %d", err);
		}
	}
}

/*
 * GPIO Interrupt Callback
 *
 * Called when LIS3DH INT1 pin goes high (motion detected).
 * Wakes main thread from sleep via semaphore.
 */
static void int1_callback(const struct device *dev, struct gpio_callback *cb,
			  uint32_t pins)
{
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);

	/* Disable interrupt to prevent re-triggering while pin is high */
	gpio_pin_interrupt_configure(dev, int1_gpio.pin, GPIO_INT_DISABLE);

	k_sem_give(&motion_sem);
}

/*
 * Configure LIS3DH for Hardware Motion Detection
 *
 * Sets up the accelerometer's internal motion detection engine:
 * - 1Hz ODR in low-power mode (~2µA)
 * - High-pass filter removes gravity, detects only acceleration changes
 * - Threshold interrupt on any axis exceeding configured mg value
 */
static int configure_hardware_motion(void)
{
	int err;
	uint8_t val;

	/* Verify WHO_AM_I */
	err = lis3dh_read_reg(LIS2DH_REG_WHO_AM_I, &val);
	if (err) {
		LOG_ERR("Failed to read WHO_AM_I: %d", err);
		return err;
	}
	if (val != LIS2DH_WHO_AM_I_VALUE) {
		LOG_ERR("Unexpected WHO_AM_I: 0x%02X (expected 0x%02X)",
			val, LIS2DH_WHO_AM_I_VALUE);
		return -ENODEV;
	}
	LOG_INF("LIS3DH detected (WHO_AM_I: 0x%02X)", val);

	/*
	 * CTRL_REG1: 1Hz ODR, low-power mode, all axes enabled
	 * - Lowest power consumption while still detecting motion
	 * - ~1 second wake latency (acceptable for presence detection)
	 */
	err = lis3dh_write_reg(LIS2DH_REG_CTRL_REG1,
			       LIS2DH_ODR_1HZ | LIS2DH_LPEN | LIS2DH_XYZ_EN);
	if (err) {
		LOG_ERR("Failed to configure CTRL_REG1: %d", err);
		return err;
	}

	/*
	 * CTRL_REG2: High-pass filter for interrupt 1
	 * - Removes DC gravity component
	 * - Only acceleration CHANGES trigger the interrupt
	 * - Lowest cutoff frequency for best motion detection
	 */
	err = lis3dh_write_reg(LIS2DH_REG_CTRL_REG2,
			       LIS2DH_HPCF_LOWEST | LIS2DH_HP_IA1);
	if (err) {
		LOG_ERR("Failed to configure CTRL_REG2: %d", err);
		return err;
	}

	/*
	 * CTRL_REG3: Route AOI1 (motion) interrupt to INT1 pin
	 */
	err = lis3dh_write_reg(LIS2DH_REG_CTRL_REG3, LIS2DH_I1_AOI1);
	if (err) {
		LOG_ERR("Failed to configure CTRL_REG3: %d", err);
		return err;
	}

	/*
	 * CTRL_REG4: Default (2g range, normal resolution)
	 * - 2g range provides 16mg/LSB for threshold calculation
	 */
	err = lis3dh_write_reg(LIS2DH_REG_CTRL_REG4, 0x00);
	if (err) {
		LOG_ERR("Failed to configure CTRL_REG4: %d", err);
		return err;
	}

	/*
	 * CTRL_REG5: Latch interrupt (INT1 stays high until INT1_SRC is read)
	 */
	err = lis3dh_write_reg(LIS2DH_REG_CTRL_REG5, BIT(3));  /* LIR_INT1 */
	if (err) {
		LOG_ERR("Failed to configure CTRL_REG5: %d", err);
		return err;
	}

	/*
	 * INT1_THS: Motion threshold
	 * - At 2g range: 1 LSB = 16mg
	 * - 48mg threshold = 3 LSBs
	 */
	uint8_t threshold_lsb = HW_MOTION_THRESHOLD_MG / 16;
	if (threshold_lsb < 1) {
		threshold_lsb = 1;
	}
	err = lis3dh_write_reg(LIS2DH_REG_INT1_THS, threshold_lsb);
	if (err) {
		LOG_ERR("Failed to configure INT1_THS: %d", err);
		return err;
	}

	/*
	 * INT1_DURATION: Samples above threshold before interrupt
	 * - 1 = immediate trigger (single sample above threshold)
	 */
	err = lis3dh_write_reg(LIS2DH_REG_INT1_DURATION, HW_MOTION_DURATION);
	if (err) {
		LOG_ERR("Failed to configure INT1_DURATION: %d", err);
		return err;
	}

	/*
	 * Read REFERENCE register to reset high-pass filter
	 * - Sets current position as "zero" reference
	 */
	err = lis3dh_read_reg(LIS2DH_REG_REFERENCE, &val);
	if (err) {
		LOG_ERR("Failed to read REFERENCE: %d", err);
		return err;
	}

	/*
	 * INT1_CFG: Enable high-event interrupt on all axes (OR combination)
	 * - Any axis exceeding threshold triggers interrupt
	 */
	err = lis3dh_write_reg(LIS2DH_REG_INT1_CFG,
			       LIS2DH_INT1_XHIE | LIS2DH_INT1_YHIE | LIS2DH_INT1_ZHIE);
	if (err) {
		LOG_ERR("Failed to configure INT1_CFG: %d", err);
		return err;
	}

	/*
	 * Clear any pending interrupts by reading INT1_SRC
	 */
	lis3dh_read_reg(LIS2DH_REG_INT1_SRC, &val);

	LOG_INF("HW motion OK: threshold=%dmg duration=%d ODR=1Hz",
		HW_MOTION_THRESHOLD_MG, HW_MOTION_DURATION);

	return 0;
}

/*
 * Clear Pending Motion Interrupt
 *
 * Reads INT1_SRC to clear the latched interrupt.
 */
static void clear_motion_interrupt(void)
{
	uint8_t val;
	lis3dh_read_reg(LIS2DH_REG_INT1_SRC, &val);
}

/*
 * Configure GPIO Interrupt for INT1 Pin
 */
static int configure_gpio_interrupt(void)
{
	int err;

	if (!gpio_is_ready_dt(&int1_gpio)) {
		LOG_ERR("INT1 GPIO not ready");
		return -ENODEV;
	}

	/* Configure as input */
	err = gpio_pin_configure_dt(&int1_gpio, GPIO_INPUT);
	if (err) {
		LOG_ERR("Failed to configure INT1 GPIO: %d", err);
		return err;
	}

	/* Setup callback */
	gpio_init_callback(&int1_cb_data, int1_callback, BIT(int1_gpio.pin));
	err = gpio_add_callback(int1_gpio.port, &int1_cb_data);
	if (err) {
		LOG_ERR("Failed to add INT1 callback: %d", err);
		return err;
	}

	/* Clear any pending LIS3DH interrupt before enabling GPIO interrupt */
	clear_motion_interrupt();
	k_msleep(10);
	clear_motion_interrupt();

	/* Enable rising edge interrupt (uses low-power SENSE via sense-edge-mask in DTS) */
	err = gpio_pin_interrupt_configure_dt(&int1_gpio, GPIO_INT_EDGE_TO_ACTIVE);
	if (err) {
		LOG_ERR("Failed to configure INT1 interrupt: %d", err);
		return err;
	}

	LOG_INF("GPIO interrupt configured on pin %d", int1_gpio.pin);
	return 0;
}

/*
 * BLE Advertising Control
 */

static int start_advertising(void)
{
	if (app_state.is_advertising) {
		return 0;
	}

	int err = bt_le_adv_start(&adv_params, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (err && err != -EALREADY) {
		LOG_ERR("Failed to start advertising: %d", err);
		return err;
	}

	app_state.is_advertising = true;
	LOG_INF(">>> BLE advertising STARTED (motion detected)");
	return 0;
}

static int stop_advertising(void)
{
	if (!app_state.is_advertising) {
		return 0;
	}

	int err = bt_le_adv_stop();
	if (err && err != -EALREADY) {
		LOG_ERR("Failed to stop advertising: %d", err);
		return err;
	}

	app_state.is_advertising = false;
	LOG_INF("<<< BLE advertising STOPPED (no motion timeout)");
	return 0;
}

/*
 * Main Application Loop
 *
 * State machine:
 * - SLEEP: MCU sleeps indefinitely waiting for motion interrupt
 * - ACTIVE: BLE advertising, check for timeout, process new motion events
 */
static void run_main_loop(void)
{
	while (1) {
		if (app_state.is_advertising) {
			/*
			 * ACTIVE MODE: Advertising, waiting for motion or timeout
			 */

			/* Wait for motion interrupt with 500ms timeout for periodic checks */
			int sem_ret = k_sem_take(&motion_sem, K_MSEC(500));
			int64_t now = k_uptime_get();

			if (sem_ret == 0) {
				/* Motion detected - reset timeout */
				app_state.last_motion_time = now;
				clear_motion_interrupt();
				LOG_DBG("Motion interrupt");
			}

			/* Re-enable GPIO interrupt */
			gpio_pin_interrupt_configure_dt(&int1_gpio, GPIO_INT_EDGE_TO_ACTIVE);

			/* Check for battery update interval (every 5 minutes) */
			int64_t battery_elapsed_sec = (now - battery.last_update_time) / 1000;
			if (battery_elapsed_sec >= BATTERY_UPDATE_INTERVAL_SEC) {
				update_battery_status();
			}

			/* Check for no-motion timeout */
			int64_t elapsed_sec = (now - app_state.last_motion_time) / 1000;
			if (elapsed_sec >= NO_MOTION_TIMEOUT_SEC) {
				stop_advertising();
			}
		} else {
			/*
			 * SLEEP MODE: Ultra-low power, waiting for motion
			 */
			LOG_INF("Entering sleep (waiting for motion)...");

			/* Clear semaphore and any pending interrupts */
			k_sem_reset(&motion_sem);
			clear_motion_interrupt();

			/* Re-enable GPIO interrupt */
			gpio_pin_interrupt_configure_dt(&int1_gpio, GPIO_INT_EDGE_TO_ACTIVE);

			/* Check if INT1 is already high (motion during setup) */
			if (gpio_pin_get_dt(&int1_gpio)) {
				LOG_INF("INT1 already high - motion pending");
				clear_motion_interrupt();
				app_state.last_motion_time = k_uptime_get();
				start_advertising();
				continue;
			}

			/* Wait indefinitely for motion interrupt */
			k_sem_take(&motion_sem, K_FOREVER);

			/* Motion detected - wake up and start advertising */
			LOG_INF("Motion detected - waking up!");
			app_state.last_motion_time = k_uptime_get();
			clear_motion_interrupt();
			start_advertising();
		}
	}
}

int main(void)
{
	int err;

	/* Brief delay for hardware stabilization */
	k_msleep(100);

	LOG_INF("BLE Beacon | %s | thresh=%dmg timeout=%ds",
		CONFIG_BT_DEVICE_NAME, HW_MOTION_THRESHOLD_MG, NO_MOTION_TIMEOUT_SEC);

	/* Get I2C device */
	i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));
	if (!device_is_ready(i2c_dev)) {
		LOG_ERR("I2C device not ready");
		return -ENODEV;
	}

	/* Configure LIS3DH for hardware motion detection */
	LOG_INF("Configuring LIS3DH...");
	err = configure_hardware_motion();
	if (err) {
		LOG_ERR("Failed to configure hardware motion detection: %d", err);
		return err;
	}

	/* Configure GPIO interrupt */
	err = configure_gpio_interrupt();
	if (err) {
		LOG_ERR("Failed to configure GPIO interrupt: %d", err);
		return err;
	}

	/* Initialize battery monitoring */
	LOG_INF("Initializing battery ADC...");
	err = init_battery_adc();
	if (err) {
		LOG_WRN("Battery ADC init failed: %d (continuing without battery monitoring)", err);
	} else {
		/* Perform initial battery reading */
		update_battery_status();
	}

	/* Initialize Bluetooth */
	LOG_INF("Initializing Bluetooth...");
	err = bt_enable(NULL);
	if (err) {
		LOG_ERR("Bluetooth init failed: %d", err);
		return err;
	}
	LOG_INF("Bluetooth ready");

	/* Initialize state */
	app_state.is_advertising = false;
	app_state.last_motion_time = 0;

	LOG_INF("Ready - move device to start advertising");

	/* Run main state machine */
	run_main_loop();

	return 0;
}
