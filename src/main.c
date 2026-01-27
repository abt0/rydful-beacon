/*
 * Power Test Step 2: Baseline + GPIO Interrupt Callback
 *
 * Tests if registering a GPIO interrupt callback affects sleep current.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/logging/log.h>
#include <hal/nrf_saadc.h>

LOG_MODULE_REGISTER(test, LOG_LEVEL_INF);

/*
 * LIS3DH Register Definitions
 */
#define LIS2DH_REG_CTRL_REG1      0x20
#define LIS2DH_REG_CTRL_REG2      0x21
#define LIS2DH_REG_CTRL_REG3      0x22
#define LIS2DH_REG_CTRL_REG4      0x23
#define LIS2DH_REG_CTRL_REG5      0x24
#define LIS2DH_REG_REFERENCE      0x26
#define LIS2DH_REG_INT1_CFG       0x30
#define LIS2DH_REG_INT1_SRC       0x31
#define LIS2DH_REG_INT1_THS       0x32
#define LIS2DH_REG_INT1_DURATION  0x33
#define LIS2DH_REG_WHO_AM_I       0x0F

#define LIS2DH_WHO_AM_I_VALUE     0x33
#define LIS2DH_ODR_1HZ            (0x01 << 4)
#define LIS2DH_LPEN               BIT(3)
#define LIS2DH_XYZ_EN             0x07
#define LIS2DH_HP_IA1             BIT(0)
#define LIS2DH_HPCF_LOWEST        (0x03 << 4)
#define LIS2DH_I1_AOI1            BIT(6)
#define LIS2DH_INT1_XHIE          BIT(1)
#define LIS2DH_INT1_YHIE          BIT(3)
#define LIS2DH_INT1_ZHIE          BIT(5)

#define HW_MOTION_THRESHOLD_MG    32
#define HW_MOTION_DURATION        2
#define NO_MOTION_TIMEOUT_SEC     10  /* Short timeout for testing */

/* ADC configuration */
#define ADC_NODE                  DT_NODELABEL(adc)
#define ADC_CHANNEL_ID            0
#define ADC_RESOLUTION_BITS       12

/* Device handles */
static const struct device *i2c_dev;
#define LIS3DH_I2C_ADDR           0x19

/* GPIO for INT1 - P0.02 */
static const struct gpio_dt_spec int1_gpio = {
	.port = DEVICE_DT_GET(DT_NODELABEL(gpio0)),
	.pin = 2,
	.dt_flags = GPIO_ACTIVE_HIGH
};

/* Semaphore for interrupt signaling */
K_SEM_DEFINE(motion_sem, 0, 1);

/* BLE advertising data */
static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL, 0xAA, 0xFE),
};

static const struct bt_data sd[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME,
		sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

/* Connectable, scannable advertising */
static struct bt_le_adv_param adv_params = BT_LE_ADV_PARAM_INIT(
	BT_LE_ADV_OPT_CONN | BT_LE_ADV_OPT_SCANNABLE,
	0x0190,  /* 250ms min */
	0x0280,  /* 400ms max */
	NULL
);

/* Application state */
static bool is_advertising = false;
static int64_t last_motion_time = 0;

/* ADC */
static const struct device *adc_dev;
static int16_t adc_buffer;

/* GPIO callback data */
static struct gpio_callback int1_cb_data;

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
 * GPIO Interrupt Callback
 */
static void int1_callback(const struct device *dev, struct gpio_callback *cb,
			  uint32_t pins)
{
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);
	
	/* Disable interrupt to prevent re-triggering while pin is high */
	gpio_pin_interrupt_configure(dev, int1_gpio.pin, GPIO_INT_DISABLE);
	
	LOG_INF(">>> INT1 callback!");
	k_sem_give(&motion_sem);
}

/*
 * Configure LIS3DH
 */
static int configure_hardware_motion(void)
{
	int err;
	uint8_t val;

	err = lis3dh_read_reg(LIS2DH_REG_WHO_AM_I, &val);
	if (err || val != LIS2DH_WHO_AM_I_VALUE) {
		return -ENODEV;
	}

	err = lis3dh_write_reg(LIS2DH_REG_CTRL_REG1,
			       LIS2DH_ODR_1HZ | LIS2DH_LPEN | LIS2DH_XYZ_EN);
	if (err) return err;

	err = lis3dh_write_reg(LIS2DH_REG_CTRL_REG2,
			       LIS2DH_HPCF_LOWEST | LIS2DH_HP_IA1);
	if (err) return err;

	err = lis3dh_write_reg(LIS2DH_REG_CTRL_REG3, LIS2DH_I1_AOI1);
	if (err) return err;

	err = lis3dh_write_reg(LIS2DH_REG_CTRL_REG4, 0x00);
	if (err) return err;

	err = lis3dh_write_reg(LIS2DH_REG_CTRL_REG5, BIT(3));
	if (err) return err;

	uint8_t threshold_lsb = HW_MOTION_THRESHOLD_MG / 16;
	if (threshold_lsb < 1) threshold_lsb = 1;
	err = lis3dh_write_reg(LIS2DH_REG_INT1_THS, threshold_lsb);
	if (err) return err;

	err = lis3dh_write_reg(LIS2DH_REG_INT1_DURATION, HW_MOTION_DURATION);
	if (err) return err;

	err = lis3dh_read_reg(LIS2DH_REG_REFERENCE, &val);
	if (err) return err;

	err = lis3dh_write_reg(LIS2DH_REG_INT1_CFG,
			       LIS2DH_INT1_XHIE | LIS2DH_INT1_YHIE | LIS2DH_INT1_ZHIE);
	if (err) return err;

	lis3dh_read_reg(LIS2DH_REG_INT1_SRC, &val);

	return 0;
}

/*
 * Clear LIS3DH interrupt by reading INT1_SRC
 */
static void clear_lis3dh_interrupt(void)
{
	uint8_t val;
	lis3dh_read_reg(LIS2DH_REG_INT1_SRC, &val);
}

/*
 * BLE Advertising Control
 */
static int start_advertising(void)
{
	if (is_advertising) {
		return 0;
	}

	int err = bt_le_adv_start(&adv_params, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (err && err != -EALREADY) {
		LOG_ERR("Adv start failed: %d", err);
		return err;
	}

	is_advertising = true;
	LOG_INF(">>> Advertising STARTED");
	return 0;
}

static int stop_advertising(void)
{
	if (!is_advertising) {
		return 0;
	}

	int err = bt_le_adv_stop();
	if (err && err != -EALREADY) {
		LOG_ERR("Adv stop failed: %d", err);
		return err;
	}

	is_advertising = false;
	LOG_INF("<<< Advertising STOPPED");
	return 0;
}

/*
 * ADC / Battery
 */
static int init_adc(void)
{
	adc_dev = DEVICE_DT_GET(ADC_NODE);
	if (!device_is_ready(adc_dev)) {
		LOG_ERR("ADC not ready");
		return -ENODEV;
	}

	struct adc_channel_cfg channel_cfg = {
		.gain = ADC_GAIN_1_6,
		.reference = ADC_REF_INTERNAL,
		.acquisition_time = ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 40),
		.channel_id = ADC_CHANNEL_ID,
		.input_positive = NRF_SAADC_INPUT_AIN1,
	};

	int err = adc_channel_setup(adc_dev, &channel_cfg);
	if (err) {
		LOG_ERR("ADC setup failed: %d", err);
		return err;
	}

	LOG_INF("ADC initialized");
	return 0;
}

static uint16_t read_battery_mv(void)
{
	struct adc_sequence sequence = {
		.channels = BIT(ADC_CHANNEL_ID),
		.buffer = &adc_buffer,
		.buffer_size = sizeof(adc_buffer),
		.resolution = ADC_RESOLUTION_BITS,
	};

	int err = adc_read(adc_dev, &sequence);
	if (err) {
		LOG_ERR("ADC read failed: %d", err);
		return 0;
	}

	/* Convert: 1/6 gain, 0.6V ref, 12-bit */
	int32_t mv = ((int32_t)adc_buffer * 600 * 6) / 4095;
	return (uint16_t)mv;
}

/*
 * Configure GPIO with interrupt callback
 */
static int configure_gpio_interrupt(void)
{
	int err;

	if (!gpio_is_ready_dt(&int1_gpio)) {
		return -ENODEV;
	}

	err = gpio_pin_configure_dt(&int1_gpio, GPIO_INPUT);
	if (err) return err;

	/* Setup and register callback */
	gpio_init_callback(&int1_cb_data, int1_callback, BIT(int1_gpio.pin));
	err = gpio_add_callback(int1_gpio.port, &int1_cb_data);
	if (err) return err;

	/* Clear any pending LIS3DH interrupt before enabling GPIO interrupt */
	clear_lis3dh_interrupt();
	k_msleep(10);
	clear_lis3dh_interrupt();

	/* Enable rising edge interrupt (will use low-power SENSE with CONFIG_GPIO_NRF_INT_EDGE_USING_SENSE) */
	err = gpio_pin_interrupt_configure_dt(&int1_gpio, GPIO_INT_EDGE_TO_ACTIVE);
	if (err) return err;

	return 0;
}

int main(void)
{
	k_msleep(100);

	LOG_INF("=== GPIO Interrupt Test ===");

	i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));
	if (!device_is_ready(i2c_dev)) {
		LOG_ERR("I2C not ready");
		return -ENODEV;
	}
	LOG_INF("I2C ready");

	if (configure_hardware_motion() != 0) {
		LOG_ERR("LIS3DH config failed");
		return -EIO;
	}
	LOG_INF("LIS3DH configured");

	if (configure_gpio_interrupt() != 0) {
		LOG_ERR("GPIO interrupt config failed");
		return -EIO;
	}
	LOG_INF("GPIO interrupt configured");

	/* Initialize Bluetooth */
	int err = bt_enable(NULL);
	if (err) {
		LOG_ERR("Bluetooth init failed: %d", err);
		return err;
	}
	LOG_INF("Bluetooth initialized (not advertising)");

	/* Initialize ADC */
	if (init_adc() != 0) {
		LOG_WRN("ADC init failed, continuing without battery monitoring");
	} else {
		uint16_t mv = read_battery_mv();
		LOG_INF("Battery: %d mV", mv);
	}

	/* Check initial pin state */
	int pin_state = gpio_pin_get_dt(&int1_gpio);
	LOG_INF("INT1 pin initial state: %d", pin_state);

	LOG_INF("Waiting for motion...");

	/* Main loop */
	while (1) {
		if (is_advertising) {
			/* Active: wait with timeout for motion or no-motion check */
			int ret = k_sem_take(&motion_sem, K_MSEC(500));
			
			if (ret == 0) {
				/* Motion detected */
				last_motion_time = k_uptime_get();
				clear_lis3dh_interrupt();
				LOG_INF("Motion (advertising)");
			}
			
			/* Check timeout */
			int64_t elapsed = (k_uptime_get() - last_motion_time) / 1000;
			if (elapsed >= NO_MOTION_TIMEOUT_SEC) {
				stop_advertising();
			}
		} else {
			/* Sleep: wait forever for motion */
			LOG_INF("Sleeping...");
			k_sem_reset(&motion_sem);
			clear_lis3dh_interrupt();
			
			k_sem_take(&motion_sem, K_FOREVER);
			
			/* Motion woke us up */
			LOG_INF("Motion detected - waking!");
			last_motion_time = k_uptime_get();
			clear_lis3dh_interrupt();
			start_advertising();
		}
		
		/* Re-enable GPIO interrupt */
		gpio_pin_interrupt_configure_dt(&int1_gpio, GPIO_INT_EDGE_TO_ACTIVE);
	}

	return 0;
}
