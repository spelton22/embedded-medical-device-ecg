#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h> 
#include <zephyr/logging/log.h>  // needs CONFIG_LOG=y in your prj.conf
#include <zephyr/drivers/adc.h> // CONFIG_ADC=y
#include <zephyr/drivers/pwm.h> // CONFIG_PWM=y
#include <zephyr/smf.h> // CONFIG_SMF=y
#include <zephyr/sys/util_macro.h>
#include "calc_cycles.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

// define macros 
#define HEARTBEAT_TOGGLE_INTERVAL_MS 500 
#define LED_BLINK_FREQ_HZ 2 // frequency of blinking for an LED
#define FREQ_UP_INC_HZ 1
#define FREQ_DOWN_INC_HZ 1 
#define ACTION_BUTTON_MIN_THRESHOLD_HZ 1
#define ACTION_BUTTON_MAX_THRESHOLD_HZ 5
#define BUFFER_ARRAY_LEN 400 // 20 samples/cycle × 20 cycles

#define ADC_DT_SPEC_GET_BY_ALIAS(adc_alias)                    \
{                                                            \
    .dev = DEVICE_DT_GET(DT_PARENT(DT_ALIAS(adc_alias))),      \
    .channel_id = DT_REG_ADDR(DT_ALIAS(adc_alias)),            \
    ADC_CHANNEL_CFG_FROM_DT_NODE(DT_ALIAS(adc_alias))          \
} 

static const struct adc_dt_spec adc_vadc = ADC_DT_SPEC_GET_BY_ALIAS(vadc);
static const struct adc_dt_spec adc_vadc_diff = ADC_DT_SPEC_GET_BY_ALIAS(diffadc);

static const struct pwm_dt_spec pwm1 = PWM_DT_SPEC_GET(DT_ALIAS(pwm1));
static const struct pwm_dt_spec pwm2 = PWM_DT_SPEC_GET(DT_ALIAS(pwm2));

static enum adc_action diff_adc_sequence_callback(const struct device *dev, const struct adc_sequence *sequence, uint16_t sampling_index);

int16_t buf;
int16_t diff_adc_buffer[BUFFER_ARRAY_LEN];

struct adc_sequence sequence = {
    .buffer = &buf,
    .buffer_size = sizeof(buf), // bytes
};

struct adc_sequence_options options = {
    .extra_samplings = BUFFER_ARRAY_LEN - 1,  // -1 b/c first sample is already in the buffer
    .interval_us = 5000,  // 5ms between samples (200 Hz sampling)
    .callback = diff_adc_sequence_callback,  // called after each sample is collected
    // .async = &async_signal,
};

struct adc_sequence sequence_diff = {
    .options = &options,  // add the options to the sequence
    .buffer = &diff_adc_buffer,  // buf is now a pointer to the first index of an array
    .buffer_size = sizeof(diff_adc_buffer),  // need to specify the size of the buffer array in bytes
        // non-global/local array scope

    /*  If the buffer is not global or local in scope, the buffer (array) name will just be a pointer to 
        the first element of the array!  The buffer_size will need to be calculated as the product of the 
        size (in bytes) of this first index and the length of the array (number of indices in the array)

    .buffer_size = BUFFER_ARRAY_LEN * sizeof(buf),  // non-global/local array scope
    */
};

// declare function prototypes
// void increase_action_led_blink_frequency(void);
// void decrease_action_led_blink_frequency(void);
// void heartbeat_blink(int64_t current_time);
void configure_buttons(void);
// void read_adc(void);
int calc_cycles(int16_t *buffer, int buffer_size);

void pwm_update_work_handler(struct k_work *work);
K_WORK_DEFINE(pwm_update_work, pwm_update_work_handler);

uint8_t button_events_bit_array = 0x0;

K_EVENT_DEFINE(button_events);
#define SLEEP_BUTTON_EVENT BIT(0)
#define FREQ_UP_BUTTON_EVENT BIT(1)
#define FREQ_DOWN_BUTTON_EVENT BIT(2)
#define RESET_BUTTON_EVENT BIT(3)

K_EVENT_DEFINE(async_diff_event);
#define ADC_DIFF_READ_EVENT BIT(0)

//ADC timer
void on_timer_interval_expiry_handler(struct k_timer *on_timer);
void off_timer_interval_expiry_handler(struct k_timer *off_timer);
void duration_timer_interval_expiry_handler(struct k_timer *duration_timer);

K_TIMER_DEFINE(on_timer, on_timer_interval_expiry_handler, NULL);
K_TIMER_DEFINE(off_timer, off_timer_interval_expiry_handler, NULL);
K_TIMER_DEFINE(duration_timer, duration_timer_interval_expiry_handler, NULL);

//timer functions
void action_timer_expiry_handler(struct k_timer *timer_action);
K_TIMER_DEFINE(timer_action, action_timer_expiry_handler, NULL);

// define globals and DT-based hardware structs
//static const struct gpio_dt_spec heartbeat_led = GPIO_DT_SPEC_GET(DT_ALIAS(heartbeat), gpios);
static const struct gpio_dt_spec sleep_button = GPIO_DT_SPEC_GET(DT_ALIAS(sleepbutton), gpios);
static const struct gpio_dt_spec reset_button = GPIO_DT_SPEC_GET(DT_ALIAS(resetbutton), gpios);
static const struct gpio_dt_spec freq_up_button = GPIO_DT_SPEC_GET(DT_ALIAS(frequpbutton), gpios);
static const struct gpio_dt_spec freq_down_button = GPIO_DT_SPEC_GET(DT_ALIAS(freqdownbutton), gpios);

static const struct gpio_dt_spec heartbeat_led = GPIO_DT_SPEC_GET(DT_ALIAS(heartbeat), gpios);
static const struct gpio_dt_spec iv_pump_led = GPIO_DT_SPEC_GET(DT_ALIAS(ivpump), gpios);
static const struct gpio_dt_spec buzzer_led = GPIO_DT_SPEC_GET(DT_ALIAS(buzzer), gpios);
static const struct gpio_dt_spec error_led = GPIO_DT_SPEC_GET(DT_ALIAS(error), gpios);

// define callback functions
void sleep_button_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
void reset_button_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
void freq_up_button_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
void freq_down_button_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins);

// initialize GPIO Callback Structs
static struct gpio_callback sleep_button_cb;  // example; need one per callback (button)  
static struct gpio_callback reset_button_cb;  // example; need one per callback (button)  
static struct gpio_callback freq_up_button_cb;  // example; need one per callback (button)  
static struct gpio_callback freq_down_button_cb;  // example; need one per callback (button)

struct led {
    int64_t time1;  // int64_t b/c time functions in Zephyr use this type
    int64_t time2;
    bool illuminated; // state of the LED (on/off)
};

struct led heartbeat_led_status = {
    .time1 = 0,
    .time2 = 0,
    .illuminated = true
};

struct led iv_pump_led_status = {
    .time1 = 0,
    .time2 = 0,
    .illuminated = true
};

struct led buzzer_led_status = {
    .time1 = 0,
    .time2 = 0,
    .illuminated = false
};

struct led error_led_status = {
    .time1 = 0,
    .time2 = 0,
    .illuminated = false
};

int heartbeat_led_event = 0;

void heartbeat_thread(void *, void *, void *);

K_THREAD_DEFINE(heartbeat_thread_id, 1024, heartbeat_thread, NULL, NULL, NULL, 5, 0, 0);

void heartbeat_thread(void *a, void *b, void *c) {
    while (1) {
        gpio_pin_set_dt(&heartbeat_led, 1);
        k_msleep(250);  // scheduler can run other tasks now
        gpio_pin_toggle_dt(&heartbeat_led);
        k_msleep(750); // scheduler can run other tasks now
        gpio_pin_toggle_dt(&heartbeat_led);
    } 
}


static const struct smf_state machine_states[];

enum machine_state { INIT, IDLE, ERROR, ADC_READ, ADC_BLINK, ADC_DIFF_READ, PWM_STEADY, PWM_STEADY_LED, PWM_SIN_READ};

struct s_object {
    /* This must be first */
    struct smf_ctx ctx;
    int last_action_led_toggle_freq ;
    int16_t last_action_led_toggle_time ;
    // int sleep_button_pressed ;
    int timer_remaining ;
    int action_led_event ;
    int error;
    // float adc_read_value ;
    float adc_read_mv ;
    int return_value ;

    float adc_on_time_ms ;
    float adc_off_time_ms ;
    int timer_done;

    int64_t time1_duration_timer;
    int64_t time2_duration_timer;
    int sampling_index ;
    uint32_t pwm_pulse_width;
    uint16_t samp_index;
} s_obj;

// s_obj.error = 0;


static void init_run(void *o)
{
    LOG_INF("System Init State");
    // check if interface is ready
    if (!device_is_ready(sleep_button.port)) {
        LOG_ERR("gpio0 interface not ready.");  // logging module output
        return ;  // exit code that will exit main()
    }
    // configure GPIO pin
    s_obj.error = gpio_pin_configure_dt(&sleep_button, GPIO_INPUT);
    if (s_obj.error < 0) {
        LOG_ERR("Cannot configure sw0 pin.");
        return ;
    }
    s_obj.error = gpio_pin_configure_dt(&reset_button, GPIO_INPUT);
    if (s_obj.error < 0) {
        LOG_ERR("Cannot configure reset button.");
        return ;
    }
    s_obj.error = gpio_pin_configure_dt(&freq_up_button, GPIO_INPUT);
    if (s_obj.error < 0) {
        LOG_ERR("Cannot configure frequency up button.");
        return ;
    }
    s_obj.error = gpio_pin_configure_dt(&freq_down_button, GPIO_INPUT);
    if (s_obj.error < 0) {
        LOG_ERR("Cannot configure frequency down button.");
        return ;
    }

    s_obj.error = gpio_pin_configure_dt(&heartbeat_led, GPIO_OUTPUT_ACTIVE);
    if (s_obj.error < 0) {
        LOG_ERR("Cannot configure heartbeat LED.");
        return ;
    }

    s_obj.error = gpio_pin_configure_dt(&iv_pump_led, GPIO_OUTPUT_ACTIVE);
    if (s_obj.error < 0) {
        LOG_ERR("Cannot configure iv pump LED.");
        return ;
    }

    s_obj.error = gpio_pin_configure_dt(&buzzer_led, GPIO_OUTPUT_INACTIVE);
    if (s_obj.error < 0) {
        LOG_ERR("Cannot configure buzzer LED.");
        return ;
    }

    s_obj.error = gpio_pin_configure_dt(&error_led, GPIO_OUTPUT_INACTIVE);
    if (s_obj.error < 0) {
        LOG_ERR("Cannot configure error LED.");
        return ;
    }

    /* Check that the ADC interface is ready */
    if (!device_is_ready(adc_vadc.dev)) {
        LOG_ERR("ADC controller device(s) not ready");
        return ;
    }

    /* Configure the ADC channel */
    s_obj.error = adc_channel_setup_dt(&adc_vadc);
    if (s_obj.error < 0) {
        LOG_ERR("Could not setup ADC channel (%d)", s_obj.error);
        return ;
    }

    s_obj.error = adc_channel_setup_dt(&adc_vadc_diff);
    if (s_obj.error < 0) {
        LOG_ERR("Could not setup ADC channel (%d)", s_obj.error);
        return ;
    }

    if (!device_is_ready(pwm1.dev)) {
        LOG_ERR("PWM device %s is not ready.", pwm1.dev->name);
        return ;
    }

    if (!device_is_ready(pwm2.dev)) {
        LOG_ERR("PWM device %s is not ready.", pwm2.dev->name);
        return ;
    }

    configure_buttons();
    // populate CB struct with information about the CB function and pin
    gpio_init_callback(&sleep_button_cb, sleep_button_callback, BIT(sleep_button.pin)); // associate callback with GPIO pin
    gpio_add_callback_dt(&sleep_button, &sleep_button_cb);

    gpio_init_callback(&reset_button_cb, reset_button_callback, BIT(reset_button.pin)); // associate callback with GPIO pin
    gpio_add_callback_dt(&reset_button, &reset_button_cb);

    gpio_init_callback(&freq_up_button_cb, freq_up_button_callback, BIT(freq_up_button.pin)); // associate callback with GPIO pin
    gpio_add_callback_dt(&freq_up_button, &freq_up_button_cb);

    gpio_init_callback(&freq_down_button_cb, freq_down_button_callback, BIT(freq_down_button.pin)); // associate callback with GPIO pin
    gpio_add_callback_dt(&freq_down_button, &freq_down_button_cb);

    k_event_init(&button_events);

    s_obj.last_action_led_toggle_freq = LED_BLINK_FREQ_HZ;
    s_obj.last_action_led_toggle_time  = 0 ;
    // s_obj.sleep_button_pressed =0;
    s_obj.timer_remaining =0;
    // int heartbeat_led_event ;
    s_obj.action_led_event =0;
    s_obj.error = 0;

    iv_pump_led_status.illuminated = 0;
    buzzer_led_status.illuminated = 0;

    s_obj.time1_duration_timer = 0;
    s_obj.time2_duration_timer = 0;

    gpio_pin_set_dt(&error_led, 0); // ensure error LED is off

    smf_set_state(SMF_CTX(&s_obj), &machine_states[IDLE]);
}

static void error_entry(void *o)
{
    LOG_INF("In error entry state.");
    gpio_pin_set_dt(&error_led, 1);

    gpio_pin_interrupt_configure_dt(&freq_up_button, GPIO_INT_DISABLE);
    gpio_pin_interrupt_configure_dt(&freq_down_button, GPIO_INT_DISABLE);
    gpio_pin_interrupt_configure_dt(&sleep_button, GPIO_INT_DISABLE);
    gpio_pin_interrupt_configure_dt(&reset_button, GPIO_INT_EDGE_TO_ACTIVE);
}

static void error_run(void *o)
{
    LOG_INF("In error run state.");

    uint32_t event = k_event_wait(&button_events, RESET_BUTTON_EVENT, true, K_FOREVER);

    if(event & RESET_BUTTON_EVENT) {
        LOG_INF("reset button pressed in error."); 
        smf_set_state(SMF_CTX(&s_obj), &machine_states[IDLE]);
    }
}

static void error_exit(void *o)
{
    LOG_INF("In error exit state.");
    gpio_pin_set_dt(&error_led, 0);
    configure_buttons();
}

static void idle_entry(void *o)
{
    LOG_INF("In idle entry state.");
    configure_buttons();
    gpio_pin_set_dt(&iv_pump_led, 0);
    LOG_INF("set gpio iv pump");
    gpio_pin_set_dt(&buzzer_led, 0);
    LOG_INF("set gpio buzzer");
    gpio_pin_set_dt(&error_led, 0);
    LOG_INF("set gpio error");
}

static void idle_run(void *o)
{
    LOG_INF("In idle run state.");
    uint32_t event = k_event_wait(&button_events, SLEEP_BUTTON_EVENT | FREQ_UP_BUTTON_EVENT | FREQ_DOWN_BUTTON_EVENT, true, K_FOREVER);

    if(event & SLEEP_BUTTON_EVENT){
        LOG_INF("SLEEP BUTTON PRESSED");
        smf_set_state(SMF_CTX(&s_obj), &machine_states[ADC_READ]);
        return;
    }
    if(event & FREQ_UP_BUTTON_EVENT){
        LOG_INF("FREQ UP BUTTON PRESSED");
        smf_set_state(SMF_CTX(&s_obj), &machine_states[PWM_SIN_READ]); // previously ADC_DIFF_READ
        return;
    }
    if(event & FREQ_DOWN_BUTTON_EVENT){
        LOG_INF("FREQ DOWN BUTTON PRESSED");
        smf_set_state(SMF_CTX(&s_obj), &machine_states[PWM_STEADY]);
        return;
    }
}

static void adc_read_entry(void *o)
{
    LOG_INF("In ADC read entry state.");
    // init adc sequence
    LOG_INF("Measuring %s (channel %d)... ", adc_vadc.dev->name, adc_vadc.channel_id);

    (void)adc_sequence_init_dt(&adc_vadc, &sequence);

    gpio_pin_interrupt_configure_dt(&freq_up_button, GPIO_INT_DISABLE);
    gpio_pin_interrupt_configure_dt(&freq_down_button, GPIO_INT_DISABLE);
    gpio_pin_interrupt_configure_dt(&sleep_button, GPIO_INT_DISABLE);
    gpio_pin_interrupt_configure_dt(&reset_button, GPIO_INT_DISABLE);
}

static void adc_read_run(void *o)
{   
    LOG_INF("In ADC read run state.");
    // aquition & check if valid
    int ret = adc_read(adc_vadc.dev, &sequence);

    if (ret < 0) {
        LOG_ERR("Could not read (%d)", s_obj.error);
        smf_set_state(SMF_CTX(&s_obj), &machine_states[ERROR]);
    } else {
        LOG_DBG("Raw ADC Buffer: %d", buf);
    }

    int32_t val_mv;  //note that adc_raw_to_millivolts_dt() takes a int32_t* to modify the value in place
    val_mv = buf;  // val_mv is now the raw ADC value
    ret = adc_raw_to_millivolts_dt(&adc_vadc, &val_mv); // remember that the vadc struct containts all the DT parameters
    s_obj.adc_read_mv = (float)val_mv;
    if (ret < 0) {
        LOG_ERR("Buffer cannot be converted to mV; returning raw buffer value.");
        smf_set_state(SMF_CTX(&s_obj), &machine_states[ERROR]);
    } else {
        LOG_INF("ADC Value (mV): %d", val_mv);
        smf_set_state(SMF_CTX(&s_obj), &machine_states[ADC_BLINK]);
    }
}

static void adc_blink_entry(void *o)
{
    LOG_INF("In ADC blink entry state.");

    gpio_pin_interrupt_configure_dt(&freq_up_button, GPIO_INT_DISABLE);
    gpio_pin_interrupt_configure_dt(&freq_down_button, GPIO_INT_DISABLE);
    gpio_pin_interrupt_configure_dt(&sleep_button, GPIO_INT_DISABLE);
    gpio_pin_interrupt_configure_dt(&reset_button, GPIO_INT_DISABLE);

    s_obj.timer_done = 0;

    LOG_INF("ADC reading (mV): %f", (double)s_obj.adc_read_mv);
    if(s_obj.adc_read_mv >= 0 ) {
        float freq  = 1.0f + (((float)s_obj.adc_read_mv * 4.0f) / 3000.0f) ; // map 0-3000 mV to 1-5 Hz 
        LOG_INF("Mapped frequency (Hz): %f", (double)freq);

        s_obj.adc_on_time_ms = (1000.0f / freq) / 10.0f; // 10% duty cycle
        s_obj.adc_off_time_ms = (1000.0f / freq) - s_obj.adc_on_time_ms;
    
        gpio_pin_set_dt(&iv_pump_led, 1);
        LOG_INF("Starting LED blink timers: ON time (ms): %f, OFF time (ms): %f", (double)s_obj.adc_on_time_ms, (double)s_obj.adc_off_time_ms);
        k_timer_start(&on_timer, K_MSEC(s_obj.adc_on_time_ms), K_NO_WAIT);
        LOG_INF("Starting duration timer for 5 seconds.");
        int64_t current_tick = k_uptime_ticks();
        s_obj.time1_duration_timer = s_obj.time2_duration_timer;
        s_obj.time2_duration_timer = k_ticks_to_ns_near64(current_tick);
        k_timer_start(&duration_timer, K_MSEC(5000), K_NO_WAIT);
    } else {
        LOG_ERR("ADC reading out of bounds for blinking.");
        smf_set_state(SMF_CTX(&s_obj), &machine_states[ERROR]);
    }

}

static void adc_blink_run(void *o)
{
    // LOG_INF("In ADC blink run state.");
    if(s_obj.timer_done == 1){
        s_obj.timer_done = 0;
        smf_set_state(SMF_CTX(&s_obj), &machine_states[IDLE]);  
        return;
    }
}

static void adc_blink_exit(void *o)
{
    LOG_INF("In ADC blink exit state.");
    configure_buttons();
    gpio_pin_set_dt(&iv_pump_led, 0);

}

static void adc_diff_entry(void *o)
{
    LOG_INF("In ADC diff entry state.");
    LOG_INF("Measuring %s (channel %d)... ", adc_vadc_diff.dev->name, adc_vadc_diff.channel_id);
    (void)adc_sequence_init_dt(&adc_vadc_diff, &sequence_diff);

    gpio_pin_interrupt_configure_dt(&freq_up_button, GPIO_INT_DISABLE);
    gpio_pin_interrupt_configure_dt(&freq_down_button, GPIO_INT_DISABLE);
    gpio_pin_interrupt_configure_dt(&sleep_button, GPIO_INT_DISABLE);
    gpio_pin_interrupt_configure_dt(&reset_button, GPIO_INT_DISABLE);
}

static void adc_diff_run(void *o)
{
    LOG_INF("In ADC diff run state.");

    int ret = adc_read_async(adc_vadc_diff.dev, &sequence_diff, NULL);
    if (ret < 0) {
        LOG_ERR("Could not read differential ADC (%d)", ret);
        smf_set_state(SMF_CTX(&s_obj), &machine_states[ERROR]);
    } else {
        LOG_DBG("Differential ADC read successful.");
    }
    LOG_INF("Raw Differential ADC Buffer first value: %d", diff_adc_buffer[0]);

    uint32_t events = k_event_wait(&async_diff_event, ADC_DIFF_READ_EVENT, true, K_FOREVER);
    if(events & ADC_DIFF_READ_EVENT){
        LOG_INF("Asynchronous ADC read complete event received.");
        LOG_HEXDUMP_INF(diff_adc_buffer, sizeof(diff_adc_buffer), "Differential ADC Samples:");
        int cycles = calc_cycles(diff_adc_buffer, BUFFER_ARRAY_LEN);
        LOG_INF("Number of cycles detected: %d", cycles);
        smf_set_state(SMF_CTX(&s_obj), &machine_states[IDLE]);
    }
}

static void adc_diff_exit(void *o)
{
    LOG_INF("In ADC diff exit state.");
    configure_buttons();
}

static void pwm_steady_entry(void *o){
    LOG_INF("In PWM steady state entry");

    gpio_pin_interrupt_configure_dt(&freq_up_button, GPIO_INT_DISABLE);
    gpio_pin_interrupt_configure_dt(&freq_down_button, GPIO_INT_DISABLE);
    gpio_pin_interrupt_configure_dt(&sleep_button, GPIO_INT_DISABLE);
    gpio_pin_interrupt_configure_dt(&reset_button, GPIO_INT_DISABLE);

    // gpio_pin_set_dt(&error_led, 1);
    (void)adc_sequence_init_dt(&adc_vadc, &sequence);
}

static void pwm_steady_read_run(void *o){
    // set the PWM duty cycle (pulse length)
    LOG_INF("in pwm stead read run state");
    LOG_INF("Measuring %s (channel %d)... ", adc_vadc.dev->name, adc_vadc.channel_id);
    int ret = adc_read(adc_vadc.dev, &sequence);

    if (ret < 0) {
        LOG_ERR("Could not read (%d)", s_obj.error);
        smf_set_state(SMF_CTX(&s_obj), &machine_states[ERROR]);
    } else {
        LOG_DBG("Raw ADC Buffer: %d", buf);
    }

    int32_t val_mv;  //note that adc_raw_to_millivolts_dt() takes a int32_t* to modify the value in place
    val_mv = buf;  // val_mv is now the raw ADC value
    ret = adc_raw_to_millivolts_dt(&adc_vadc, &val_mv); // remember that the vadc struct containts all the DT parameters
    s_obj.adc_read_mv = (float)val_mv;
    if (ret < 0) {
        LOG_ERR("Buffer cannot be converted to mV; returning raw buffer value.");
        smf_set_state(SMF_CTX(&s_obj), &machine_states[ERROR]);
    } else {
        LOG_INF("ADC Value (mV): %d", val_mv);
        smf_set_state(SMF_CTX(&s_obj), &machine_states[PWM_STEADY_LED]);
    }
}

static void pwm_steady_on_entry(void *o){
    LOG_INF("in pwm steady on entry");

    s_obj.timer_done = 0;

    LOG_INF("ADC reading (mV): %f", (double)s_obj.adc_read_mv);
    if(s_obj.adc_read_mv >= 0 ) {
        float period = (float)s_obj.adc_read_mv / 3000.0f ;
        LOG_INF("Mapped period: %f", (double)period);

        int err;
        // period = 1.0f/100.0f;
        err = pwm_set_pulse_dt(&pwm1, pwm1.period*period);
        LOG_INF("full period = %d", pwm1.period);
        LOG_INF("period of pwm = %f\n", (double)(pwm1.period*period));

        if (err) {
            LOG_ERR("Could not set pwm1 driver.");
            smf_set_state(SMF_CTX(&s_obj), &machine_states[ERROR]);
        } 

        LOG_INF("Starting duration timer for 5 seconds.");
        int64_t current_tick = k_uptime_ticks();
        s_obj.time1_duration_timer = s_obj.time2_duration_timer;
        s_obj.time2_duration_timer = k_ticks_to_ns_near64(current_tick);
        k_timer_start(&duration_timer, K_MSEC(5000), K_NO_WAIT);
        // smf_set_state(SMF_CTX(&s_obj), &machine_states[IDLE]);
    } else {
        LOG_ERR("ADC reading out of bounds for blinking.");
        smf_set_state(SMF_CTX(&s_obj), &machine_states[ERROR]);
    }
}

static void pwm_steady_on_run(void *o){
    LOG_INF("in pwm steady on run state");
    if(s_obj.timer_done == 1){
        s_obj.timer_done = 0;
        smf_set_state(SMF_CTX(&s_obj), &machine_states[IDLE]);  
        return;
    }
}

static void pwm_steady_on_exit(void *o) {
    LOG_INF("in pwm steady on exit state");
    configure_buttons();
    pwm_set_pulse_dt(&pwm1, 0);
}

static void pwm_sin_read_entry(void *o) {
    LOG_INF("in pwm sin read entry state");
    s_obj.error = 0;
    s_obj.samp_index = 0;
    s_obj.pwm_pulse_width = 0;
    LOG_INF("Measuring %s (channel %d)... ", adc_vadc_diff.dev->name, adc_vadc_diff.channel_id);
    (void)adc_sequence_init_dt(&adc_vadc_diff, &sequence_diff);

    gpio_pin_interrupt_configure_dt(&freq_up_button, GPIO_INT_DISABLE);
    gpio_pin_interrupt_configure_dt(&freq_down_button, GPIO_INT_DISABLE);
    gpio_pin_interrupt_configure_dt(&sleep_button, GPIO_INT_DISABLE);
    gpio_pin_interrupt_configure_dt(&reset_button, GPIO_INT_DISABLE);
}

static void pwm_sin_read_run(void *o) {
    LOG_INF("in pwm sin read run state");
    int ret = adc_read_async(adc_vadc_diff.dev, &sequence_diff, NULL);
    if (ret < 0) {
        LOG_ERR("Could not read differential ADC (%d)", ret);
        smf_set_state(SMF_CTX(&s_obj), &machine_states[ERROR]);
    } else {
        LOG_DBG("Differential ADC read successful.");
    }
    LOG_INF("Raw Differential ADC Buffer first value: %d", diff_adc_buffer[0]);

    uint32_t events = k_event_wait(&async_diff_event, ADC_DIFF_READ_EVENT, true, K_FOREVER);
    if(events & ADC_DIFF_READ_EVENT){
        LOG_INF("Asynchronous ADC read complete event received.");
        
        if(s_obj.error == 1){
            smf_set_state(SMF_CTX(&s_obj), &machine_states[ERROR]);
        } else {
            smf_set_state(SMF_CTX(&s_obj), &machine_states[IDLE]);
        }
    }
}

static void pwm_sin_read_exit(void *o) {
    LOG_INF("in pwm sin read exit state");
    configure_buttons();
    pwm_set_pulse_dt(&pwm2, 0);
}

// define states for state machine
static const struct smf_state machine_states[] = {
    [INIT] = SMF_CREATE_STATE(NULL, init_run, NULL, NULL, NULL),
    [IDLE] = SMF_CREATE_STATE(idle_entry, idle_run, NULL, NULL, NULL),
    [ADC_READ] = SMF_CREATE_STATE(adc_read_entry, adc_read_run, NULL, NULL, NULL),
    [ADC_BLINK] = SMF_CREATE_STATE(adc_blink_entry, adc_blink_run, adc_blink_exit, NULL, NULL), 
    [ADC_DIFF_READ] = SMF_CREATE_STATE(adc_diff_entry, adc_diff_run, adc_diff_exit, NULL, NULL),
    [ERROR] = SMF_CREATE_STATE(error_entry, error_run, error_exit, NULL, NULL),
    [PWM_STEADY] = SMF_CREATE_STATE(pwm_steady_entry, pwm_steady_read_run, NULL, NULL, NULL),
    [PWM_STEADY_LED] = SMF_CREATE_STATE(pwm_steady_on_entry, pwm_steady_on_run, pwm_steady_on_exit, NULL, NULL),
    [PWM_SIN_READ] = SMF_CREATE_STATE(pwm_sin_read_entry, pwm_sin_read_run, pwm_sin_read_exit, NULL, NULL), 
};

int main (void){
    int32_t ret;
    smf_set_initial(SMF_CTX(&s_obj), &machine_states[INIT]);

    while(1) {
        /* State machine terminates if a non-zero value is returned */
        // int events = k_event_wait(&button_events, SLEEP_BUTTON_EVENT | FREQ_DOWN_BUTTON_EVENT | FREQ_DOWN_BUTTON_EVENT | RESET_BUTTON_EVENT, true, K_MSEC(1000));
        ret = smf_run_state(SMF_CTX(&s_obj));

        if (ret) {
            smf_set_terminate(SMF_CTX(&s_obj), ret);
            break;
        }
        k_msleep(1000);
    }
}

// define callback functions
void sleep_button_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    k_event_post(&button_events, SLEEP_BUTTON_EVENT);
}

void reset_button_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    k_event_post(&button_events, RESET_BUTTON_EVENT);
}

void freq_up_button_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    k_event_post(&button_events, FREQ_UP_BUTTON_EVENT);
}

void freq_down_button_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    k_event_post(&button_events, FREQ_DOWN_BUTTON_EVENT);
}

void configure_buttons(void) {
    s_obj.error = gpio_pin_interrupt_configure_dt(&freq_up_button, GPIO_INT_EDGE_TO_ACTIVE); 
    if (s_obj.error < 0) {
        LOG_ERR("Cannot attach callback to sw1.");
    }

    s_obj.error = gpio_pin_interrupt_configure_dt(&freq_down_button, GPIO_INT_EDGE_TO_ACTIVE); 
    if (s_obj.error < 0) {
        LOG_ERR("Cannot attach callback to sw2.");
    }

    s_obj.error = gpio_pin_interrupt_configure_dt(&sleep_button, GPIO_INT_EDGE_TO_ACTIVE); 
    if (s_obj.error < 0) {
        LOG_ERR("Cannot attach callback to sw0.");
    }

    s_obj.error = gpio_pin_interrupt_configure_dt(&reset_button, GPIO_INT_EDGE_TO_ACTIVE); 
    if (s_obj.error < 0) {
        LOG_ERR("Cannot attach callback to sw3.");
    }
    
    LOG_INF("finished configuring buttons!!");
}

void on_timer_interval_expiry_handler(struct k_timer *on_timer){
    // LOG_INF("ON timer expired. Starting OFF timer.");
    gpio_pin_set_dt(&iv_pump_led, 0);
    k_timer_start(&off_timer, K_MSEC(s_obj.adc_off_time_ms), K_NO_WAIT);
}

void off_timer_interval_expiry_handler(struct k_timer *off_timer){
    // LOG_INF("OFF timer expired. Starting ON timer.");
    gpio_pin_set_dt(&iv_pump_led, 1);
    k_timer_start(&on_timer, K_MSEC(s_obj.adc_on_time_ms), K_NO_WAIT);
}

void duration_timer_interval_expiry_handler(struct k_timer *duration_timer){
    // LOG_INF("Duration timer expired. Stopping ON and OFF timers.");
    s_obj.timer_done = 1;
    k_timer_stop(&on_timer);
    k_timer_stop(&off_timer);
    gpio_pin_set_dt(&iv_pump_led, 0);
    
    int64_t current_tick = k_uptime_ticks();
    s_obj.time1_duration_timer = s_obj.time2_duration_timer;
    s_obj.time2_duration_timer = k_ticks_to_ns_near64(current_tick);

    int64_t duration_ns = s_obj.time2_duration_timer - s_obj.time1_duration_timer;

    LOG_INF("5 Second Duration Timer time = %lld ns", duration_ns);
    LOG_INF("5 Second Duration Timer time = %lld s", duration_ns / 1000000000);
}

void pwm_update_work_handler(struct k_work *work) {
    int32_t val_mv = diff_adc_buffer[s_obj.samp_index];

    int ret = adc_raw_to_millivolts_dt(&adc_vadc_diff, &val_mv); // remember that the vadc struct containts all the DT parameters
    
    if( ret < 0){
        s_obj.error = 1;
    }

    s_obj.adc_read_mv = (float)val_mv;  

    // -1000 --> 1000 , 0 -> 20000 
        // can increase acquition time or take average of a few samples 
    float period = (val_mv + 1000.0) / 2000.0f;
    LOG_INF("Mapped period: %f", (double)period);
    if(period > 1.0f) {
        period = 1.0f;
    }
    if(period < 0.0f){
        period = 0.0f;
    }

    int err = pwm_set_pulse_dt(&pwm2, pwm2.period* period);

    if (err) {
        LOG_ERR("PWM set failed");
    }
    
    k_msleep(1);
    
    s_obj.samp_index++;
    // LOG_INF("index=%d\n", s_obj.samp_index);
    if (s_obj.samp_index == BUFFER_ARRAY_LEN - 1) {
        s_obj.samp_index = 0;
    }
}

static enum adc_action diff_adc_sequence_callback(const struct device *dev, const struct adc_sequence *sequence, uint16_t sampling_index)
{
    if(sampling_index == BUFFER_ARRAY_LEN-1){
        k_event_post(&async_diff_event, ADC_DIFF_READ_EVENT);
        return ADC_ACTION_FINISH;
    } 
    k_work_submit(&pwm_update_work);

    return ADC_ACTION_CONTINUE;
}