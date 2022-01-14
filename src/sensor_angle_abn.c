// Support for querying encoders based on ABN signals
//
// Wrecklab BV

#include "basecmd.h" // oid_alloc
#include "board/misc.h" // timer_read_time
#include "board/gpio.h" // gpio_out_write
#include "board/irq.h" // irq_disable
#include "command.h" // DECL_COMMAND
#include "sched.h" // DECL_TASK
#include "spicmds.h" // spidev_transfer

enum { SA_CUI_AMT102 };

DECL_ENUMERATION("abn_angle_type", "amt102", SA_CUI_AMT102);

enum { TCODE_ERROR = 0xff };

// flags (states?)
enum { SE_OVERFLOW, SE_SCHEDULE, SE_SPI_TIME, SE_CRC, SE_DUP, SE_NO_ANGLE };

struct abn_angle {
    struct timer timer;
    struct gpio_in pin_a;
    struct gpio_in pin_b;
    uint16_t sequence;
    uint8_t flags, data_count, time_shift, overflow;

    TIM_TypeDef* htim2;
    // add timer and channels from A,B,N pins
    // add ACC reg
    uint8_t data[48];
};

enum {
    SA_PENDING = 1<<2,
};

static struct task_wake angle_wake;

// Event handler that wakes abn_angle_task() periodically
static uint_fast8_t
angle_event(struct timer *timer)
{
    struct abn_angle *abna = container_of(timer, struct abn_angle, timer);
    uint8_t flags = abna->flags;
    if (abna->flags & SA_PENDING)
        abna->overflow++;
    else
        abna->flags = flags | SA_PENDING;
    sched_wake_task(&angle_wake);
    abna->timer.waketime += abna->rest_ticks;
    return SF_RESCHEDULE;
}

// WL : command to setup the encoder (aka abn_angle) in the MCU 
// using the parameters passed from py
void
command_config_abn_angle(uint32_t *args)
{
    // inspiration from pwmcmds.c 
    /*struct gpio_pwm pin = gpio_pwm_setup(args[1], args[2], args[3]);
    struct pwm_out_s *p = oid_alloc(args[0], command_config_pwm_out
                                    , sizeof(*p));
    p->pin = pin;
    ...
    p->timer.func = pwm_event; */
    
    struct gpio_in pin_a = gpio_in_setup(args[1], 0);
    struct gpio_in pin_b = gpio_in_setup(args[2], 0);
    struct abn_angle *abna = oid_alloc(args[0], command_config_abn_angle
                                     , sizeof(*abna));
    abna->pin_a = pin_a
    abna->pin_b = pin_b
    abna->timer.func = angle_event;

    TIMx_ARR

    if (abna->htim2->CR1 & TIM_CR1_CEN) {
        if (p->timer->PSC != (uint16_t) prescaler) {
            shutdown("PWM already programmed at different speed");
        }
    } else {
        abna->htim2->PSC = 0;
        abna->htim2->ARR = 8192;

        abna->htim2->TIMx_SMCR 
    }
    // TODO now configure hw timer i.e. TI1 and TI2 on TIM2 (PA0,PA1).. 
    //no prescaler, encoder mode T1 and T2 


}
DECL_COMMAND(command_config_abn_angle,
             "config_abn_angle oid=%c pin_a=%c pin_b=%c");

// Report local measurement buffer
static void
angle_report(struct abn_angle *abna, uint8_t oid)
{
    sendf("abn_angle_data oid=%c sequence=%hu data=%*s"
          , oid, abna->sequence, abna->data_count, abna->data);
    abna->data_count = 0;
    abna->sequence++;
}

// Send abn_angle_data message if buffer is full
static void
angle_check_report(struct abn_angle *abna, uint8_t oid)
{
    if (abna->data_count + 3 > ARRAY_SIZE(abna->data))
        angle_report(abna, oid);
}

// Add an entry to the measurement buffer
// TODO what the hell is doing?
static void
angle_add(struct abn_angle *abna, uint_fast8_t tcode, uint_fast16_t data)
{
    abna->data[abna->data_count] = tcode;
    abna->data[abna->data_count + 1] = data;
    abna->data[abna->data_count + 2] = data >> 8;
    abna->data_count += 3;
}

// Add an error indicator to the measurement buffer
static void
angle_add_error(struct spi_angle *sa, uint_fast8_t error_code)
{
    angle_add(sa, TCODE_ERROR, error_code);
}

// Add a measurement to the buffer
static void
angle_add_data(struct spi_angle *sa, uint32_t stime, uint32_t mtime
               , uint_fast16_t angle)
{
    uint32_t tdiff = mtime - stime;
    if (sa->time_shift)
        tdiff = (tdiff + (1<<(sa->time_shift - 1))) >> sa->time_shift;
    if (tdiff >= TCODE_ERROR) {
        angle_add_error(sa, SE_SCHEDULE);
        return;
    }
    angle_add(sa, tdiff, angle);
}

// amt102 sensor query
static void
amt102_query(struct spi_angle *sa, uint32_t stime)
{
    uint8_t msg[2] = { 0x32, 0x00 };
    uint32_t mtime1 = timer_read_time();
    spidev_transfer(sa->spi, 1, sizeof(msg), msg);
    uint32_t mtime2 = timer_read_time();
    // Data is latched on first sclk edge of response
    if (mtime2 - mtime1 > MAX_SPI_READ_TIME)
        angle_add_error(sa, SE_SPI_TIME);
    else if (msg[0] & 0x80)
        angle_add_error(sa, SE_CRC);
    else
        angle_add_data(sa, stime, mtime1, (msg[0] << 9) | (msg[1] << 1));
}


void
command_query_abn_angle(uint32_t *args)
{
    uint8_t oid = args[0];
    struct abn_angle *abna = oid_lookup(oid, command_config_abn_angle);

    sched_del_timer(&abna->timer);
    abna->flags = 0;
    if (!args[2]) {
        // End measurements
        if (sa->data_count)
            angle_report(sa, oid);
        sendf("spi_angle_end oid=%c sequence=%hu", oid, sa->sequence);
        return;
    }
    // Start new measurements query
    sa->timer.waketime = args[1];
    sa->rest_ticks = args[2];
    sa->sequence = 0;
    sa->data_count = 0;
    sa->time_shift = args[3];
    sched_add_timer(&sa->timer);
}
DECL_COMMAND(command_query_spi_angle,
             "query_spi_angle oid=%c clock=%u rest_ticks=%u time_shift=%c");

void
command_spi_angle_transfer(uint32_t *args)
{
    uint8_t oid = args[0];
    struct spi_angle *sa = oid_lookup(oid, command_config_spi_angle);
    uint8_t data_len = args[1];
    uint8_t *data = command_decode_ptr(args[2]);
    uint32_t mtime;
    uint_fast8_t chip = sa->chip_type;
    if (chip == SA_CHIP_TLE5012B) {
        // Latch data (data is latched on rising CS of a NULL message)
        struct gpio_out cs_pin = spidev_get_cs_pin(sa->spi);
        gpio_out_write(cs_pin, 0);
        udelay(1);
        irq_disable();
        gpio_out_write(cs_pin, 1);
        mtime = timer_read_time();
        irq_enable();
        spidev_transfer(sa->spi, 1, data_len, data);
    } else {
        uint32_t mtime1 = timer_read_time();
        spidev_transfer(sa->spi, 1, data_len, data);
        uint32_t mtime2 = timer_read_time();
        if (mtime2 - mtime1 > MAX_SPI_READ_TIME)
            data_len = 0;
        if (chip == SA_CHIP_AS5047D)
            mtime = mtime2;
        else
            mtime = mtime1;
    }
    sendf("spi_angle_transfer_response oid=%c clock=%u response=%*s"
          , oid, mtime, data_len, data);
}
DECL_COMMAND(command_spi_angle_transfer, "spi_angle_transfer oid=%c data=%*s");



// Background task that performs measurements
void
abn_angle_task(void)
{
    if (!sched_check_wake(&angle_wake))
        return;
    uint8_t oid;
    struct abn_angle *abna;
    foreach_oid(oid, abna, command_config_abn_angle) {
        uint_fast8_t flags = abna->flags;
        if (!(flags & SA_PENDING))
            continue;
        irq_disable();
        uint32_t stime = abna->timer.waketime;
        uint_fast8_t overflow = abna->overflow;
        abna->flags = 0;
        abna->overflow = 0;
        irq_enable();
        stime -= abna->rest_ticks;
        while (overflow--) {
            angle_add_error(abna, SE_OVERFLOW);
            angle_check_report(abna, oid);
        }
        
        // query the sensor
        atm102_query(abna, stime);

        angle_check_report(sa, oid);
    }
}
DECL_TASK(abn_angle_task);
