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

// WL cheating ... FIX: make it generic, we just want the stm32 #defines 
#include "stm32/internal.h"

// WL just copied, maybe there's a better value for this
#define MAX_ABN_READ_TIME timer_from_us(50)

// WL not reall useful like this, maybe turn it into a  choise of the TIM/pin to use
enum { SA_CUI_AMT102 };
DECL_ENUMERATION("abn_angle_type", "amt102", SA_CUI_AMT102);

enum { TCODE_ERROR = 0xff };

// flags (states?)
enum { SE_OVERFLOW, SE_SCHEDULE, SE_SPI_TIME, SE_CRC, SE_DUP, SE_NO_ANGLE };

struct abn_angle {
    struct timer timer;
    TIM_TypeDef *htim;

    struct gpio_in pin_a;
    struct gpio_in pin_b;

    uint16_t sequence;
    uint32_t rest_ticks;
    uint8_t flags, data_count, time_shift, overflow;

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
#define GPIO_PIN_0 ((uint16_t)0x0001)  /* Pin 0 selected    */
#define GPIO_PIN_1 ((uint16_t)0x0002)  /* Pin 1 selected    */
#define GPIO_MODE_AF_PP   0x00000002U   /*!< Alternate Function Push Pull Mode */
#define  GPIO_NOPULL        0x00000000U   /*!< No Pull-up or Pull-down activation  */
#define  GPIO_SPEED_FREQ_LOW         0x00000000U  /*!< IO works at 2 MHz, please refer to the product datasheet */
#define GPIO_AF1_TIM2          ((uint8_t)0x01)  /* TIM2 Alternate Function mapping */
#define GPIO_MODE             0x00000003U
#define GPIO_OUTPUT_TYPE      0x00000010U
#define GPIO_NUMBER           16U

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
    abna->pin_a = pin_a;
    abna->pin_b = pin_b;
    abna->timer.func = angle_event;

    
    /**TIM2 GPIO Configuration PA0-WKUP - TIM2_CH1, PA1 - TIM2_CH2 */
    // see Reference Manual F411 sec 8.3.9
    GPIO_TypeDef *porta = GPIOA;  
    uint16_t pins = GPIO_PIN_0|GPIO_PIN_1; 

    for(uint32_t pin = 0U; pin < GPIO_NUMBER; pin++) {
        if( (pins & (0x01U << pin)) ) {
            //temp = porta->PUPDR;
            porta->PUPDR &= ~(GPIO_PUPDR_PUPDR0 << (pin * 2U));
            porta->PUPDR |= (GPIO_NOPULL << (pin * 2U));
            porta->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED0 << (pin * 2U));
            porta->OSPEEDR |= (GPIO_SPEED_FREQ_LOW << (pin * 2U));
            //Configure the IO Output Type
            porta->OTYPER &= ~(GPIO_OTYPER_OT_0 << pin) ;
            porta->OTYPER |= (((GPIO_MODE_AF_PP & GPIO_OUTPUT_TYPE) >> 4U) << pin);
            porta->AFR[pin >> 3U] &= ~(0xFU << ((uint32_t)(pin & 0x07U) * 4U)) ;
            porta->AFR[pin >> 3U] |= ((uint32_t)(GPIO_AF1_TIM2) << (((uint32_t)pin & 0x07U) * 4U));
            porta->MODER &= ~(GPIO_MODER_MODER0 << (pin * 2U));
            porta->MODER |= ( (GPIO_MODE_AF_PP & GPIO_MODE) << (pin * 2U));
        }
    }
    
    // TODO -- FIX setup pins, from config and the board/gpio.h functions
    // enable timer on gpio
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;     // kinda from __HAL_RCC_TIM2_CLK_ENABLE();
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;    // kinda from __HAL_RCC_GPIOA_CLK_ENABLE();
    //gpio_clock_enable(GPIO_TypeDef *regs)
    //c->pin = gpio_in_setup(args[1], args[2]);

    
    // timer

    abna->htim = TIM2;
    // SMCR (Slave Mode Control Register): set encoder mode 
    abna->htim->SMCR &= ~TIM_SMCR_SMS;
    abna->htim->SMCR |= (TIM_SMCR_SMS_1 | TIM_SMCR_SMS_0);
    // CCER (): configure signal polarity
    abna->htim->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC2P);
    // CCMR1 (Capture/Compare Mode Register 1) : filter
    abna->htim->CCMR1 &= ~( (TIM_CCMR1_CC1S | TIM_CCMR1_IC1F) |
                            (TIM_CCMR1_CC2S | TIM_CCMR1_IC2F) );
    abna->htim->CCMR1 |= (  (TIM_CCMR1_CC1S_1 | TIM_CCMR1_IC1F_0) |
                            (TIM_CCMR1_CC2S_1 | TIM_CCMR1_IC2F_0) );
    // enable timer/counter
    abna->htim->PSC = 0;
    abna->htim->ARR = 8192;
    abna->htim->CR1 |= TIM_CR1_CEN; 

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
angle_add_error(struct abn_angle *abna, uint_fast8_t error_code)
{
    angle_add(abna, TCODE_ERROR, error_code);
}

// Add a measurement to the buffer
static void
angle_add_data(struct abn_angle *abna, uint32_t stime, uint32_t mtime
               , uint_fast16_t angle)
{
    uint32_t tdiff = mtime - stime;
    if (abna->time_shift)
        tdiff = (tdiff + (1<<(abna->time_shift - 1))) >> abna->time_shift;
    if (tdiff >= TCODE_ERROR) {
        angle_add_error(abna, SE_SCHEDULE);
        return;
    }
    angle_add(abna, tdiff, angle);
}

// amt102 sensor query
static void
amt102_query(struct abn_angle *abna, uint32_t stime)
{
    uint8_t msg[2] = { 0x32, 0x00 };
    uint32_t mtime1 = timer_read_time();
    
    //spidev_transfer(sa->spi, 1, sizeof(msg), msg);
    uint32_t mtime2 = timer_read_time();
    // Data is latched on first sclk edge of response
    if (mtime2 - mtime1 > MAX_ABN_READ_TIME)
        angle_add_error(abna, SE_SPI_TIME);
    //else if (msg[0] & 0x80)
    //    angle_add_error(sa, SE_CRC);
    else
        angle_add_data(abna, stime, mtime1, (msg[0] << 9) | (msg[1] << 1));
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
        if (abna->data_count)
            angle_report(abna, oid);
        sendf("abn_angle_end oid=%c sequence=%hu", oid, abna->sequence);
        return;
    }
    // Start new measurements query
    abna->timer.waketime = args[1];
    abna->rest_ticks = args[2];
    abna->sequence = 0;
    abna->data_count = 0;
    abna->time_shift = args[3];
    sched_add_timer(&abna->timer);
}
DECL_COMMAND(command_query_abn_angle,
             "query_abn_angle oid=%c clock=%u rest_ticks=%u time_shift=%c");


// WL originally this was transferring data from the sensor to our buffer, via SPI
// .. I guess here we just read the TIM counter to the buffer instead
void
command_abn_angle_transfer(uint32_t *args)
{
    uint8_t oid = args[0];
    struct abn_angle *abna = oid_lookup(oid, command_config_abn_angle);

    uint8_t data_len = args[1];
    uint8_t *data = command_decode_ptr(args[2]);
    uint32_t mtime;

    // WL read CNT register here, fake data_len to mimic SPI
    uint32_t mtime1 = timer_read_time();
    data = abna->htim->CNT;
    data_len = 1;
    uint32_t mtime2 = timer_read_time();

    if (mtime2 - mtime1 > MAX_ABN_READ_TIME)
        data_len = 0;
    else 
        mtime = mtime1;
    
    sendf("abn_angle_transfer_response oid=%c clock=%u response=%*s"
          , oid, mtime, data_len, data);
}
DECL_COMMAND(command_abn_angle_transfer, "abn_angle_transfer oid=%c data=%*s");



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
        amt102_query(abna, stime);

        angle_check_report(abna, oid);
    }
}
DECL_TASK(abn_angle_task);
