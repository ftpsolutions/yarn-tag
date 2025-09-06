#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(app, LOG_LEVEL_INF);

#define LIS_NODE DT_NODELABEL(lis2dh12)
#if !DT_NODE_EXISTS(LIS_NODE)
#error "LIS2DH12 node missing. Check board overlay."
#endif

static const struct spi_dt_spec lis =
    SPI_DT_SPEC_GET(LIS_NODE, SPI_WORD_SET(8) | SPI_TRANSFER_MSB, 0);

static const struct gpio_dt_spec lis_irq =
    GPIO_DT_SPEC_GET_OR(LIS_NODE, irq_gpios, {0});

#define LED_RED_NODE   DT_ALIAS(led0)
#define LED_GREEN_NODE DT_ALIAS(led1)
#define LED_BLUE_NODE  DT_ALIAS(led2)
static const struct gpio_dt_spec led_red   = GPIO_DT_SPEC_GET(LED_RED_NODE, gpios);
static const struct gpio_dt_spec led_green = GPIO_DT_SPEC_GET(LED_GREEN_NODE, gpios);
static const struct gpio_dt_spec led_blue  = GPIO_DT_SPEC_GET(LED_BLUE_NODE, gpios);

#define SW0_NODE DT_ALIAS(sw0)
#if !DT_NODE_HAS_STATUS(SW0_NODE, okay)
#warning "No sw0 alias in DT; DFU button disabled"
#else
static const struct gpio_dt_spec dfu_btn = GPIO_DT_SPEC_GET(SW0_NODE, gpios);
#endif

#define REG_CTRL1       0x20
#define REG_CTRL3       0x22
#define REG_CTRL4       0x23
#define REG_CTRL5       0x24
#define REG_INT1_CFG    0x30
#define REG_INT1_SRC    0x31
#define REG_INT1_THS    0x32
#define REG_INT1_DUR    0x33

#define LIS_SPI_READ    0x80
#define LIS_SPI_AUTOINC 0x40

static uint8_t mfg_payload[] = { 0xFF, 0xFF, 0x01, 0x00 };
static const char devname[] = "YarnBeacon";
static struct bt_data ad_nonconn[] = {
    BT_DATA(BT_DATA_MANUFACTURER_DATA, mfg_payload, sizeof(mfg_payload)),
};

static struct bt_data ad_conn[] = {
    BT_DATA(BT_DATA_NAME_COMPLETE, devname, sizeof(devname)-1),
    BT_DATA(BT_DATA_MANUFACTURER_DATA, mfg_payload, sizeof(mfg_payload)),
};

static const struct bt_le_adv_param adv_slow = {
    .options = BT_LE_ADV_OPT_USE_IDENTITY,
    .interval_min = 0x0800, /* ~1.28 s */
    .interval_max = 0x1000, /* ~2.56 s */
};

static const struct bt_le_adv_param adv_fast = {
    .options = BT_LE_ADV_OPT_USE_IDENTITY,
    .interval_min = 0x00A0, /* 100 ms */
    .interval_max = 0x00F0, /* 150 ms */
};

static const struct bt_le_adv_param adv_conn = {
    .options = BT_LE_ADV_OPT_CONNECTABLE | BT_LE_ADV_OPT_USE_IDENTITY,
    .interval_min = 0x00A0, /* 100 ms */
    .interval_max = 0x00F0, /* 150 ms */
};

static struct k_work_delayable slow_adv_work;

static struct k_work_delayable conn_timeout_work;
static struct k_work_delayable dfu_timeout_work;

static void conn_timeout(struct k_work *work)
{
    ARG_UNUSED(work);
    (void)bt_le_adv_stop();
    (void)bt_le_adv_start(&adv_slow, ad_nonconn, ARRAY_SIZE(ad_nonconn), NULL, 0);
    gpio_pin_set_dt(&led_blue, 0);
}

static void dfu_timeout(struct k_work *work)
{
    ARG_UNUSED(work);
    (void)bt_le_adv_stop();
    (void)bt_le_adv_start(&adv_slow, ad_nonconn, ARRAY_SIZE(ad_nonconn), NULL, 0);
    gpio_pin_set_dt(&led_green, 0);
}


static int lis_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = { reg & ~(LIS_SPI_READ | LIS_SPI_AUTOINC), val };
    const struct spi_buf buf = { .buf = tx, .len = sizeof(tx) };
    const struct spi_buf_set txs = { .buffers = &buf, .count = 1 };
    return spi_write_dt(&lis, &txs);
}

static int lis_read_reg(uint8_t reg, uint8_t *val)
{
    uint8_t tx = reg | LIS_SPI_READ;
    const struct spi_buf txb = { .buf = &tx, .len = 1 };
    const struct spi_buf rxb = { .buf = val, .len = 1 };
    const struct spi_buf_set txs = { .buffers = &txb, .count = 1 };
    const struct spi_buf_set rxs = { .buffers = &rxb, .count = 1 };
    return spi_transceive_dt(&lis, &txs, &rxs);
}

static int lis_config_high_g(uint8_t fs_sel, uint8_t ths_raw, uint8_t dur)
{
    int err;
    err = lis_write_reg(REG_CTRL1, 0x27);
    if (err) return err;
    uint8_t ctrl4 = (fs_sel & 0x3) << 4;
    err = lis_write_reg(REG_CTRL4, ctrl4);
    if (err) return err;
    err = lis_write_reg(REG_CTRL5, 0x08);
    if (err) return err;
    err = lis_write_reg(REG_INT1_THS, ths_raw);
    if (err) return err;
    err = lis_write_reg(REG_INT1_DUR, dur);
    if (err) return err;
    err = lis_write_reg(REG_INT1_CFG, 0x2A);
    if (err) return err;
    err = lis_write_reg(REG_CTRL3, 0x40);
    return err;
}

static void lis_clear_int(void)
{
    uint8_t src;
    (void)lis_read_reg(REG_INT1_SRC, &src);
}

static void switch_to_slow_adv(struct k_work *work)
{
    ARG_UNUSED(work);
    (void)bt_le_adv_stop();
    (void)bt_le_adv_start(&adv_slow, ad_nonconn, ARRAY_SIZE(ad_nonconn), NULL, 0);
    gpio_pin_set_dt(&led_red, 0);
}


static void start_connectable_window(int seconds)
{
    /* Cancel any pending transitions that could fight us */
    k_work_cancel_delayable(&slow_adv_work);
    k_work_cancel_delayable(&conn_timeout_work);
    k_work_cancel_delayable(&dfu_timeout_work);

    (void)bt_le_adv_stop();
    (void)bt_le_adv_start(&adv_conn, ad_conn, ARRAY_SIZE(ad_conn), NULL, 0);
    gpio_pin_set_dt(&led_blue, 1);
    k_work_reschedule(&conn_timeout_work, K_SECONDS(seconds));
}

static void start_dfu_window(int seconds)
{
    /* Long-press: longer connectable window for configuration */
    k_work_cancel_delayable(&slow_adv_work);
    k_work_cancel_delayable(&conn_timeout_work);
    k_work_cancel_delayable(&dfu_timeout_work);

    (void)bt_le_adv_stop();
    (void)bt_le_adv_start(&adv_conn, ad_conn, ARRAY_SIZE(ad_conn), NULL, 0);
    gpio_pin_set_dt(&led_green, 1);
    k_work_reschedule(&dfu_timeout_work, K_SECONDS(seconds));
}

static void set_flag_and_burst(uint8_t value)
{
    mfg_payload[3] = value;
    (void)bt_le_adv_update_data(ad_nonconn, ARRAY_SIZE(ad_nonconn), NULL, 0);
    (void)bt_le_adv_stop();
    (void)bt_le_adv_start(&adv_fast, ad_nonconn, ARRAY_SIZE(ad_nonconn), NULL, 0);
    gpio_pin_set_dt(&led_red, value ? 1 : 0);
    k_work_reschedule(&slow_adv_work, K_SECONDS(45));
}


static struct gpio_callback irq_cb;
static struct gpio_callback btn_cb;

static void lis_int_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    ARG_UNUSED(dev); ARG_UNUSED(cb); ARG_UNUSED(pins);
    lis_clear_int();
    set_flag_and_burst(1);
}


static int64_t btn_pressed_ms = -1;
#define LONG_PRESS_MS 2000
#define SHORT_MIN_MS   50

static void dfu_button_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    ARG_UNUSED(cb); ARG_UNUSED(pins);
#if DT_NODE_HAS_STATUS(SW0_NODE, okay)
    int level = gpio_pin_get_dt(&dfu_btn);
    int64_t now = k_uptime_get();
    if (level > 0) {
        /* pressed */
        btn_pressed_ms = now;
    } else {
        /* released */
        if (btn_pressed_ms >= 0) {
            int64_t dur = now - btn_pressed_ms;
            btn_pressed_ms = -1;
            if (dur >= LONG_PRESS_MS) {
                start_dfu_window(180);
            } else if (dur >= SHORT_MIN_MS) {
                start_connectable_window(60);
            }
        }
    }
#endif
}

int main(void)
{
    int err;

    if (!device_is_ready(lis.bus))       { LOG_ERR("SPI bus not ready"); return -ENODEV; }
    if (!device_is_ready(lis_irq.port))  { LOG_ERR("IRQ GPIO not ready"); return -ENODEV; }
    if (!device_is_ready(led_red.port))  { LOG_ERR("LEDs not ready"); return -ENODEV; }
#if DT_NODE_HAS_STATUS(SW0_NODE, okay)
    if (!device_is_ready(dfu_btn.port))  { LOG_ERR("DFU button not ready"); return -ENODEV; }
#endif

    gpio_pin_configure_dt(&led_red, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&led_green, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&led_blue, GPIO_OUTPUT_INACTIVE);

    /* INT1 input + edge trigger */
    gpio_pin_configure_dt(&lis_irq, GPIO_INPUT | GPIO_PULL_UP);
    gpio_pin_interrupt_configure_dt(&lis_irq, GPIO_INT_EDGE_TO_ACTIVE);
    gpio_init_callback(&irq_cb, lis_int_isr, BIT(lis_irq.pin));
    gpio_add_callback(lis_irq.port, &irq_cb);
#if DT_NODE_HAS_STATUS(SW0_NODE, okay)
    gpio_pin_interrupt_configure_dt(&dfu_btn, GPIO_INT_EDGE_BOTH);
    gpio_init_callback(&btn_cb, dfu_button_isr, BIT(dfu_btn.pin));
    gpio_add_callback(dfu_btn.port, &btn_cb);
#endif

#if DT_NODE_HAS_STATUS(SW0_NODE, okay)
    /* External 100k pulldown present; no internal pull */
    gpio_pin_configure_dt(&dfu_btn, GPIO_INPUT);
#endif

    /* BLE start (slow adverts) */
    err = bt_enable(NULL);
    if (err) { LOG_ERR("bt_enable failed (%d)", err); return err; }
    k_work_init_delayable(&slow_adv_work, switch_to_slow_adv);
    k_work_init_delayable(&conn_timeout_work, conn_timeout);
    k_work_init_delayable(&dfu_timeout_work, dfu_timeout);
    (void)bt_le_adv_start(&adv_slow, ad_nonconn, ARRAY_SIZE(ad_nonconn), NULL, 0);


    /* Configure LIS2DH12 */
    err = lis_config_high_g(3, 0x14, 0x01);
    if (err) { LOG_ERR("LIS2DH12 cfg failed (%d)", err); }
    else     { LOG_INF("Ready: advertising + LIS2DH12 (SPI) + DFU-button"); }

    for (;;) {
        k_sleep(K_SECONDS(10));
        if (mfg_payload[3]) {
            set_flag_and_burst(0);
        }
    }
}
