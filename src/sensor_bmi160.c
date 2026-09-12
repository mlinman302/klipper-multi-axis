// Support for gathering data from a BMI160 accelerometer/gyroscope
//
// Copyright (C) 2025  Francisco Stephens <francisco.stephens.g@gmail.com>
// Copyright (C) 2026  Klipper multi-axis contributors
//
// This file may be distributed under the terms of the GNU GPLv3 license.
#include <string.h> // memcpy
#include "autoconf.h" // CONFIG_WANT_SPI
#include "board/gpio.h" // irq_disable
#include "board/irq.h" // irq_disable
#include "board/misc.h" // timer_read_time
#include "basecmd.h" // oid_alloc
#include "command.h" // DECL_COMMAND
#include "sched.h" // DECL_TASK
#include "sensor_bulk.h" // sensor_bulk_report
#include "spicmds.h" // spidev_transfer
#include "i2ccmds.h" // i2cdev_s
#include "trigger_analog.h" // trigger_analog_update

#define BMI_AR_DATAX0 0x12
#define BMI_AM_READ   0x80
#define BMI_FIFO_STATUS 0x22
#define BMI_FIFO_DATA 0x24

// A headerless fifo frame is six bytes with only the accelerometer
// enabled, and twelve with the gyroscope enabled as well (gyro first -
// the fifo stores sensor data in data-register order).  Klipper's
// FixedFreqReader timestamps by counting, so every bulk message must
// carry exactly MAX_BULK_MSG_SIZE (51) / frame_size whole frames:
// 51/6 = 8 frames and 51/12 = 4 frames, both of which are 48 bytes.
#define BYTES_PER_BLOCK 48

struct bmi160 {
    struct timer timer;
    uint32_t rest_ticks;
    union {
        struct spidev_s *spi;
        struct i2cdev_s *i2c;
    };
    struct trigger_analog *ta;
    uint8_t bus_type;
    uint8_t flags;
    uint8_t bytes_per_frame;
    uint8_t frame_offset;
    uint16_t fifo_bytes_pending;
    struct sensor_bulk sb;
};

enum {
    BMI_PENDING = 1<<0,
};

enum {
    SPI_SERIAL, I2C_SERIAL,
};

DECL_ENUMERATION("bus_oid_type", "spi", SPI_SERIAL);
DECL_ENUMERATION("bus_oid_type", "i2c", I2C_SERIAL);

static struct task_wake bmi160_wake;

// Event handler that wakes bmi160_task() periodically
static uint_fast8_t
bmi160_event(struct timer *timer)
{
    struct bmi160 *ax = container_of(timer, struct bmi160, timer);
    ax->flags |= BMI_PENDING;
    sched_wake_task(&bmi160_wake);
    return SF_DONE;
}

void
command_config_bmi160(uint32_t *args)
{
    struct bmi160 *ax = oid_alloc(args[0], command_config_bmi160
                                   , sizeof(*ax));
    ax->timer.func = bmi160_event;
    if (args[3] == 0 || BYTES_PER_BLOCK % args[3])
        shutdown("bytes_per_frame must divide the bulk block size");
    ax->bytes_per_frame = args[3];

    switch (args[2]) {
        case SPI_SERIAL:
            if (CONFIG_WANT_SPI) {
                ax->spi = spidev_oid_lookup(args[1]);
                ax->bus_type = SPI_SERIAL;
                break;
            } else {
                shutdown("bus_type spi unsupported");
            }
        case I2C_SERIAL:
            if (CONFIG_WANT_I2C) {
               ax->i2c = i2cdev_oid_lookup(args[1]);
               ax->bus_type = I2C_SERIAL;
               break;
            } else {
                shutdown("bus_type i2c unsupported");
            }
        default:
            shutdown("bus_type invalid");
    }
}
DECL_COMMAND(command_config_bmi160, "config_bmi160 oid=%c"
                " bus_oid=%c bus_oid_type=%c bytes_per_frame=%c");

// Attach an mcu-side threshold detector to one channel of each frame.
// frame_offset is the byte offset of the 16-bit channel to watch, so it
// selects both the sensor and the axis - see docs/BMI160_IMU.md.
void
command_bmi160_attach_trigger_analog(uint32_t *args)
{
    struct bmi160 *ax = oid_lookup(args[0], command_config_bmi160);
    if (args[2] + 2 > ax->bytes_per_frame)
        shutdown("frame_offset outside of bmi160 frame");
    ax->frame_offset = args[2];
    ax->ta = trigger_analog_oid_lookup(args[1]);
}
DECL_COMMAND(command_bmi160_attach_trigger_analog,
             "bmi160_attach_trigger_analog oid=%c trigger_analog_oid=%c"
             " frame_offset=%c");

// Helper code to reschedule the bmi160_event() timer
static void
bmi160_reschedule_timer(struct bmi160 *ax)
{
    irq_disable();
    ax->timer.waketime = timer_read_time() + ax->rest_ticks;
    sched_add_timer(&ax->timer);
    irq_enable();
}

// Update local status tracking from newly read fifo status register
static void
update_fifo_status(struct bmi160 *ax, uint16_t fifo_bytes)
{
    // BMI160 FIFO can hold up to 1024 bytes
    if (fifo_bytes > 1024)
        ax->sb.possible_overflows++;
    ax->fifo_bytes_pending = fifo_bytes;
}

// Query fifo status register
static void
query_fifo_status(struct bmi160 *ax)
{
    uint16_t fifo_bytes = 0;
    if (CONFIG_WANT_SPI && ax->bus_type == SPI_SERIAL) {
        uint8_t fifo[3] = { BMI_FIFO_STATUS | BMI_AM_READ, 0x00, 0x00 };
        spidev_transfer(ax->spi, 1, sizeof(fifo), fifo);
        fifo_bytes = (fifo[2] << 8) | fifo[1];
    } else if (CONFIG_WANT_I2C && ax->bus_type == I2C_SERIAL) {
        uint8_t fifo_reg[1] = {BMI_FIFO_STATUS};
        uint8_t fifo_val[2];
        int ret = i2c_dev_read(ax->i2c, sizeof(fifo_reg), fifo_reg
                               , sizeof(fifo_val), fifo_val);
        i2c_shutdown_on_err(ret);
        fifo_bytes = (fifo_val[1] << 8) | fifo_val[0];
    }
    update_fifo_status(ax, fifo_bytes);
}

// Read 8 samples from FIFO via SPI
static void
read_fifo_block_spi(struct bmi160 *ax)
{
    uint8_t msg[BYTES_PER_BLOCK + 1] = {0};
    msg[0] = BMI_FIFO_DATA | BMI_AM_READ;

    spidev_transfer(ax->spi, 1, sizeof(msg), msg);
    memcpy(ax->sb.data, &msg[1], BYTES_PER_BLOCK);
}

// Read 8 samples from FIFO via i2c
static void
read_fifo_block_i2c(struct bmi160 *ax)
{
    uint8_t msg_reg[] = {BMI_FIFO_DATA};

    int ret = i2c_dev_read(ax->i2c, sizeof(msg_reg), msg_reg
                           , BYTES_PER_BLOCK, ax->sb.data);
    i2c_shutdown_on_err(ret);
}

// Feed the watched channel of every frame in the block to the detector
static void
update_trigger(struct bmi160 *ax)
{
    uint8_t bytes_per_frame = ax->bytes_per_frame;
    uint8_t *data = ax->sb.data;
    uint8_t i;
    for (i = ax->frame_offset; i < BYTES_PER_BLOCK; i += bytes_per_frame) {
        int16_t value = (int16_t)((data[i + 1] << 8) | data[i]);
        trigger_analog_update(ax->ta, value);
    }
}

// Read from fifo and transmit data to host
static void
read_fifo_block(struct bmi160 *ax, uint8_t oid)
{
    if (CONFIG_WANT_SPI && ax->bus_type == SPI_SERIAL)
        read_fifo_block_spi(ax);
    else if (CONFIG_WANT_I2C && ax->bus_type == I2C_SERIAL)
        read_fifo_block_i2c(ax);
    // Detect before reporting - the host transfer is not in the path of
    // a homing decision
    if (ax->ta)
        update_trigger(ax);
    ax->sb.data_count = BYTES_PER_BLOCK;
    sensor_bulk_report(&ax->sb, oid);
    ax->fifo_bytes_pending -= BYTES_PER_BLOCK;
}

// Query accelerometer data
static void
bmi160_query(struct bmi160 *ax, uint8_t oid)
{
    if (ax->fifo_bytes_pending < BYTES_PER_BLOCK)
        query_fifo_status(ax);

    if (ax->fifo_bytes_pending >= BYTES_PER_BLOCK)
        read_fifo_block(ax, oid);

    // check if we need to run the task again (more packets in fifo?)
    if (ax->fifo_bytes_pending >= BYTES_PER_BLOCK) {
        // More data in fifo - wake this task again
        sched_wake_task(&bmi160_wake);
    } else {
        // Sleep until next check time
        ax->flags &= ~BMI_PENDING;
        bmi160_reschedule_timer(ax);
    }
}

void
command_query_bmi160(uint32_t *args)
{
    struct bmi160 *ax = oid_lookup(args[0], command_config_bmi160);

    sched_del_timer(&ax->timer);
    ax->flags = 0;
    if (!args[1])
        // End measurements
        return;

    // Start new measurements query
    ax->rest_ticks = args[1];
    ax->fifo_bytes_pending = 0;
    sensor_bulk_reset(&ax->sb);
    bmi160_reschedule_timer(ax);
}
DECL_COMMAND(command_query_bmi160, "query_bmi160 oid=%c rest_ticks=%u");

void
command_query_bmi160_status(uint32_t *args)
{
    struct bmi160 *ax = oid_lookup(args[0], command_config_bmi160);
    uint32_t time1 = 0;
    uint32_t time2 = 0;
    uint16_t fifo_bytes = 0;

    if (CONFIG_WANT_SPI && ax->bus_type == SPI_SERIAL) {
        uint8_t fifo[3] = { BMI_FIFO_STATUS | BMI_AM_READ, 0x00, 0x00 };
        time1 = timer_read_time();
        spidev_transfer(ax->spi, 1, sizeof(fifo), fifo);
        time2 = timer_read_time();
        fifo_bytes = (fifo[2] << 8) | fifo[1];
    } else if (CONFIG_WANT_I2C && ax->bus_type == I2C_SERIAL) {
        uint8_t fifo_reg[1] = {BMI_FIFO_STATUS};
        uint8_t fifo_val[2];
        time1 = timer_read_time();
        int ret = i2c_dev_read(ax->i2c, sizeof(fifo_reg), fifo_reg
                               , sizeof(fifo_val), fifo_val);
        time2 = timer_read_time();
        i2c_shutdown_on_err(ret);
        fifo_bytes = (fifo_val[1] << 8) | fifo_val[0];
    }
    update_fifo_status(ax, fifo_bytes);

    sensor_bulk_status(&ax->sb, args[0], time1, time2-time1
                       , ax->fifo_bytes_pending);
}
DECL_COMMAND(command_query_bmi160_status, "query_bmi160_status oid=%c");

void
bmi160_task(void)
{
    if (!sched_check_wake(&bmi160_wake))
        return;
    uint8_t oid;
    struct bmi160 *ax;
    foreach_oid(oid, ax, command_config_bmi160) {
        uint_fast8_t flags = ax->flags;
        if (flags & BMI_PENDING)
            bmi160_query(ax, oid);
    }
}
DECL_TASK(bmi160_task);
