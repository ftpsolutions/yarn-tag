#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <string.h>

/* --- LIS2DH12 accelerometer configuration ------------------------------ */

#define LIS_NODE DT_NODELABEL(lis2dh12)
#if !DT_NODE_EXISTS(LIS_NODE)
#error "LIS2DH12 node missing. Check board overlay."
#endif

static const struct spi_dt_spec lis =
    SPI_DT_SPEC_GET(LIS_NODE, SPI_WORD_SET(8) | SPI_TRANSFER_MSB, 0);

static const struct gpio_dt_spec lis_irq =
    GPIO_DT_SPEC_GET_OR(LIS_NODE, irq_gpios, {0});

#define REG_CTRL1    0x20
#define REG_CTRL3    0x22
#define REG_CTRL4    0x23
#define REG_CTRL5    0x24
#define REG_INT1_CFG 0x30
#define REG_INT1_SRC 0x31
#define REG_INT1_THS 0x32
#define REG_INT1_DUR 0x33

#define LIS_SPI_READ    0x80
#define LIS_SPI_AUTOINC 0x40

/* --- LEDs --------------------------------------------------------------- */

#define LED_RED_NODE   DT_ALIAS(led0)
#define LED_GREEN_NODE DT_ALIAS(led1)
#define LED_BLUE_NODE  DT_ALIAS(led2)
static const struct gpio_dt_spec led_red   = GPIO_DT_SPEC_GET(LED_RED_NODE, gpios);
static const struct gpio_dt_spec led_green = GPIO_DT_SPEC_GET(LED_GREEN_NODE, gpios);
static const struct gpio_dt_spec led_blue  = GPIO_DT_SPEC_GET(LED_BLUE_NODE, gpios);

/* --- User button ------------------------------------------------------- */

#define SW0_NODE DT_ALIAS(sw0)
#if !DT_NODE_HAS_STATUS(SW0_NODE, okay)
#warning "No sw0 alias in DT; button disabled"
#else
static const struct gpio_dt_spec user_btn = GPIO_DT_SPEC_GET(SW0_NODE, gpios);
#endif

/* --- Beacon payload ---------------------------------------------------- */

/* Manufacturer data layout: [CompanyID(2)][state][battery] */
#define MFG_STATE_IDX 2
#define MFG_BATT_IDX  3
static uint8_t mfg_payload[] = { 0xFF, 0xFF, 0x00, 0x00 };

static uint8_t triggered_state;
static uint8_t battery_state = 1; /* 1 = good, 0 = low */
static uint32_t trigger_count;
static int64_t last_trigger_ms;

/* Advertised name. Updated via phone app; default placeholder. */
static char devname[20] = "YMT-UNNAMED";

static struct bt_data ad_nonconn[] = {
    BT_DATA(BT_DATA_MANUFACTURER_DATA, mfg_payload, sizeof(mfg_payload)),
};

static struct bt_data ad_conn[2]; /* filled at runtime */

/* Advertising parameters */
static const struct bt_le_adv_param adv_slow = {
    .options = BT_LE_ADV_OPT_USE_IDENTITY,
    .interval_min = 0x1F40, /* 5 s */
    .interval_max = 0x1F40, /* 5 s */
};

static const struct bt_le_adv_param adv_fast = {
    .options = BT_LE_ADV_OPT_USE_IDENTITY,
    .interval_min = 0x00A0, /* 100 ms */
    .interval_max = 0x00F0, /* 150 ms */
};

static const struct bt_le_adv_param adv_conn = {
    .options = BT_LE_ADV_OPT_CONNECTABLE | BT_LE_ADV_OPT_USE_IDENTITY,
    .interval_min = 0x00A0,
    .interval_max = 0x00F0,
};

/* Work items ------------------------------------------------------------- */

static struct k_work_delayable slow_adv_work;
static struct k_work_delayable conn_timeout_work;
static struct k_work_delayable session_timeout_work;
static struct k_work_delayable battery_work;

struct flash_ctx {
    struct k_work_delayable work;
    const struct gpio_dt_spec *led;
};

static void flash_cycle(struct k_work *work)
{
    struct flash_ctx *ctx = CONTAINER_OF(work, struct flash_ctx, work);
    gpio_pin_set_dt(ctx->led, 1);
    k_msleep(100);
    gpio_pin_set_dt(ctx->led, 0);
    k_work_schedule(&ctx->work, K_SECONDS(5));
}

static inline void flash_ctx_init(struct flash_ctx *ctx,
                                  const struct gpio_dt_spec *led)
{
    k_work_init_delayable(&ctx->work, flash_cycle);
    ctx->led = led;
}

static inline void flash_ctx_start(struct flash_ctx *ctx)
{
    k_work_cancel_delayable(&ctx->work);
    k_work_schedule(&ctx->work, K_NO_WAIT);
}

static inline void flash_ctx_stop(struct flash_ctx *ctx)
{
    k_work_cancel_delayable(&ctx->work);
    gpio_pin_set_dt(ctx->led, 0);
}

/* Power-on self-test LED sequence: red, green, blue */
static void led_post(void)
{
    const struct gpio_dt_spec *seq[] = { &led_red, &led_green, &led_blue };
    for (size_t i = 0; i < ARRAY_SIZE(seq); i++) {
        gpio_pin_set_dt(seq[i], 1);
        k_msleep(100);
        gpio_pin_set_dt(seq[i], 0);
        k_msleep(100);
    }
}

static void flash_green_once(struct k_work *work)
{
    ARG_UNUSED(work);
    gpio_pin_set_dt(&led_green, 1);
    k_msleep(100);
    gpio_pin_set_dt(&led_green, 0);
}

static void flash_green_twice(struct k_work *work)
{
    ARG_UNUSED(work);
    for (int i = 0; i < 2; i++) {
        gpio_pin_set_dt(&led_green, 1);
        k_msleep(100);
        gpio_pin_set_dt(&led_green, 0);
        k_msleep(100);
    }
}

static struct flash_ctx red_flash;
static struct flash_ctx blue_flash;
static struct k_work green_flash_work;
static struct k_work green_double_work;

static bool connectable_mode;
static struct bt_conn *curr_conn;

static int lis_config_high_g(uint8_t fs_sel, uint8_t ths_raw, uint8_t dur);

/* --- GATT configuration service ---------------------------------------- */

#define BT_UUID_CFG_SERVICE_VAL   BT_UUID_128_ENCODE(0x4e9a0001, 0x9f31, 0x415b, 0x8f82, 0x94dfc6201ca8)
#define BT_UUID_ACCEL_THRESH_VAL  BT_UUID_128_ENCODE(0x4e9a0002, 0x9f31, 0x415b, 0x8f82, 0x94dfc6201ca8)
#define BT_UUID_CFG_SERVICE       BT_UUID_DECLARE_128(BT_UUID_CFG_SERVICE_VAL)
#define BT_UUID_ACCEL_THRESH      BT_UUID_DECLARE_128(BT_UUID_ACCEL_THRESH_VAL)

static uint8_t accel_thresh = 0x14;

static ssize_t read_thresh(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                           void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset,
                             attr->user_data, sizeof(accel_thresh));
}

static ssize_t write_thresh(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                            const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
    if (len != 1 || offset != 0) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    uint8_t *val = attr->user_data;
    *val = *(const uint8_t *)buf;
    (void)lis_config_high_g(3, *val, 0x01);
    k_work_submit(&green_flash_work);
    return len;
}

BT_GATT_SERVICE_DEFINE(cfg_svc,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_CFG_SERVICE),
    BT_GATT_CHARACTERISTIC(BT_UUID_ACCEL_THRESH,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
                           BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
                           read_thresh, write_thresh, &accel_thresh),
);

/* --- Helpers ------------------------------------------------------------ */

static void update_adv_payload(void)
{
    mfg_payload[MFG_STATE_IDX] = triggered_state;
    mfg_payload[MFG_BATT_IDX]  = battery_state;

    if (connectable_mode) {
        ad_conn[1].data = mfg_payload;
        ad_conn[1].data_len = sizeof(mfg_payload);
        (void)bt_le_adv_update_data(ad_conn, ARRAY_SIZE(ad_conn), NULL, 0);
    } else {
        (void)bt_le_adv_update_data(ad_nonconn, ARRAY_SIZE(ad_nonconn), NULL, 0);
    }
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
    connectable_mode = false;
    (void)bt_le_adv_stop();
    (void)bt_le_adv_start(&adv_slow, ad_nonconn, ARRAY_SIZE(ad_nonconn), NULL, 0);
    flash_ctx_stop(&red_flash);
    flash_ctx_stop(&blue_flash);
}

static void conn_timeout(struct k_work *work)
{
    ARG_UNUSED(work);
    switch_to_slow_adv(NULL);
}

static void session_timeout(struct k_work *work)
{
    ARG_UNUSED(work);
    if (curr_conn) {
        bt_conn_disconnect(curr_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
    }
}

/* Battery check placeholder */
static void check_battery(struct k_work *work)
{
    ARG_UNUSED(work);
    /* TODO: Read actual battery level via ADC */
    battery_state = 1;
    update_adv_payload();
    k_work_reschedule(&battery_work, K_MINUTES(1));
}

static void start_connectable_window(int seconds)
{
    k_work_cancel_delayable(&slow_adv_work);
    k_work_cancel_delayable(&conn_timeout_work);

    connectable_mode = true;

    ad_conn[0].type = BT_DATA_NAME_COMPLETE;
    ad_conn[0].data = devname;
    ad_conn[0].data_len = strlen(devname);
    ad_conn[1].type = BT_DATA_MANUFACTURER_DATA;
    ad_conn[1].data = mfg_payload;
    ad_conn[1].data_len = sizeof(mfg_payload);

    (void)bt_le_adv_stop();
    (void)bt_le_adv_start(&adv_conn, ad_conn, ARRAY_SIZE(ad_conn), NULL, 0);
    flash_ctx_stop(&blue_flash);
    flash_ctx_start(&red_flash);
    k_work_reschedule(&conn_timeout_work, K_SECONDS(seconds));
}

static void set_triggered_and_burst(bool value)
{
    triggered_state = value ? 1 : 0;
    if (triggered_state) {
        trigger_count++;
        last_trigger_ms = k_uptime_get();
    }
    update_adv_payload();
    (void)bt_le_adv_stop();
    (void)bt_le_adv_start(&adv_fast, ad_nonconn, ARRAY_SIZE(ad_nonconn), NULL, 0);
    k_work_reschedule(&slow_adv_work, K_SECONDS(45));
}

static void reset_trigger(void)
{
    triggered_state = 0;
    update_adv_payload();
}

/* --- ISRs --------------------------------------------------------------- */

static struct gpio_callback irq_cb;
static struct gpio_callback btn_cb;

static void lis_int_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    ARG_UNUSED(dev); ARG_UNUSED(cb); ARG_UNUSED(pins);
    lis_clear_int();
    set_triggered_and_burst(true);
}

static int64_t btn_pressed_ms = -1;
#define LONG_PRESS_MS 2000
#define SHORT_PRESS_MAX_MS 1000
#define SHORT_MIN_MS 50

static void user_button_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    ARG_UNUSED(dev); ARG_UNUSED(cb); ARG_UNUSED(pins);
#if DT_NODE_HAS_STATUS(SW0_NODE, okay)
    int level = gpio_pin_get_dt(&user_btn);
    int64_t now = k_uptime_get();
    if (level > 0) {
        btn_pressed_ms = now;
    } else {
        if (btn_pressed_ms >= 0) {
            int64_t dur = now - btn_pressed_ms;
            btn_pressed_ms = -1;
            if (dur >= LONG_PRESS_MS) {
                start_connectable_window(60);
            } else if (dur >= SHORT_MIN_MS && dur <= SHORT_PRESS_MAX_MS) {
                reset_trigger();
                k_work_submit(&green_double_work);
            }
        }
    }
#endif
}

/* --- Connection callbacks ---------------------------------------------- */

static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        return;
    }
    curr_conn = bt_conn_ref(conn);
    k_work_cancel_delayable(&conn_timeout_work);
    k_work_reschedule(&session_timeout_work, K_MINUTES(2));
    flash_ctx_stop(&red_flash);
    flash_ctx_start(&blue_flash);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    ARG_UNUSED(reason);
    if (curr_conn) {
        bt_conn_unref(curr_conn);
        curr_conn = NULL;
    }
    k_work_cancel_delayable(&session_timeout_work);
    conn_timeout(NULL);
}

static struct bt_conn_cb conn_cbs = {
    .connected = connected,
    .disconnected = disconnected,
};

/* --- Main ---------------------------------------------------------------- */

int main(void)
{
    int err;

    if (!device_is_ready(lis.bus))       { return -ENODEV; }
    if (!device_is_ready(lis_irq.port))  { return -ENODEV; }
    if (!device_is_ready(led_red.port))  { return -ENODEV; }
#if DT_NODE_HAS_STATUS(SW0_NODE, okay)
    if (!device_is_ready(user_btn.port)) { return -ENODEV; }
#endif

    gpio_pin_configure_dt(&led_red, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&led_green, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&led_blue, GPIO_OUTPUT_INACTIVE);

    led_post();

    gpio_pin_configure_dt(&lis_irq, GPIO_INPUT | GPIO_PULL_UP);
    gpio_pin_interrupt_configure_dt(&lis_irq, GPIO_INT_EDGE_TO_ACTIVE);
    gpio_init_callback(&irq_cb, lis_int_isr, BIT(lis_irq.pin));
    gpio_add_callback(lis_irq.port, &irq_cb);

#if DT_NODE_HAS_STATUS(SW0_NODE, okay)
    gpio_pin_configure_dt(&user_btn, GPIO_INPUT | GPIO_PULL_UP);
    gpio_init_callback(&btn_cb, user_button_isr, BIT(user_btn.pin));
    gpio_add_callback(user_btn.port, &btn_cb);
    gpio_pin_interrupt_configure_dt(&user_btn, GPIO_INT_EDGE_BOTH);
#endif

    err = bt_enable(NULL);
    if (err) {
        return err;
    }

    bt_set_name(devname);
    bt_conn_cb_register(&conn_cbs);

    k_work_init_delayable(&slow_adv_work, switch_to_slow_adv);
    k_work_init_delayable(&conn_timeout_work, conn_timeout);
    k_work_init_delayable(&session_timeout_work, session_timeout);
    k_work_init_delayable(&battery_work, check_battery);
    flash_ctx_init(&red_flash, &led_red);
    flash_ctx_init(&blue_flash, &led_blue);
    k_work_init(&green_flash_work, flash_green_once);
    k_work_init(&green_double_work, flash_green_twice);

    update_adv_payload();
    (void)bt_le_adv_start(&adv_slow, ad_nonconn, ARRAY_SIZE(ad_nonconn), NULL, 0);
    k_work_reschedule(&battery_work, K_MINUTES(1));

    err = lis_config_high_g(3, accel_thresh, 0x01);
    (void)err;

    for (;;) {
        k_sleep(K_FOREVER);
    }
}

