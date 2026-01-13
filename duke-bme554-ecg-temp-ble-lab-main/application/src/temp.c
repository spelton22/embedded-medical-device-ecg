#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h> 
#include <zephyr/logging/log.h>  // needs CONFIG_LOG=y in your prj.conf
#include <zephyr/drivers/adc.h> // CONFIG_ADC=y
// #include <zephyr/drivers/pwm.h> // CONFIG_PWM=y
#include <zephyr/smf.h> // CONFIG_SMF=y
#include <zephyr/sys/util_macro.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

// define macros 
#define HEARTBEAT_TOGGLE_INTERVAL_MS 500 
#define LED_BLINK_FREQ_HZ 2 // frequency of blinking for an LED
#define FREQ_UP_INC_HZ 1
#define FREQ_DOWN_INC_HZ 1 
#define ACTION_BUTTON_MIN_THRESHOLD_HZ 1
#define ACTION_BUTTON_MAX_THRESHOLD_HZ 5
#define BUFFER_ARRAY_LEN 50  // number of samples to store in buffer

#define ADC_DT_SPEC_GET_BY_ALIAS(adc_alias)                    \
{                                                            \
    .dev = DEVICE_DT_GET(DT_PARENT(DT_ALIAS(adc_alias))),      \
    .channel_id = DT_REG_ADDR(DT_ALIAS(adc_alias)),            \
    ADC_CHANNEL_CFG_FROM_DT_NODE(DT_ALIAS(adc_alias))          \
}                                                            \

static const struct adc_dt_spec adc_vadc = ADC_DT_SPEC_GET_BY_ALIAS(vadc);

int16_t buf_adc;

struct adc_sequence_options options = {
    .extra_samplings = BUFFER_ARRAY_LEN - 1,  // -1 b/c first sample is already in the buffer
    .interval_us = 100,  // 100 us between samples
    // .callback = NULL,  // called after each sample is collected
};

struct adc_sequence sequence = {
    .options = &options,
    .buffer = &buf_adc,
    .buffer_size = BUFFER_ARRAY_LEN * sizeof(buf_adc), // bytes
};


// declare function prototypes
void increase_action_led_blink_frequency(void);
void decrease_action_led_blink_frequency(void);
void heartbeat_blink(int64_t current_time);
void configure_buttons(void);
void read_adc(void);

uint8_t button_events_bit_array = 0x0;

K_EVENT_DEFINE(button_events);
#define SLEEP_BUTTON_EVENT BIT(0)
#define FREQ_UP_BUTTON_EVENT BIT(1)
#define FREQ_DOWN_BUTTON_EVENT BIT(2)
#define RESET_BUTTON_EVENT BIT(3)

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
        
        // int64_t current_tick = k_uptime_ticks();
        // heartbeat_led_status.time1 = heartbeat_led_status.time2;
        // heartbeat_led_status.time2 = k_ticks_to_ns_near64(current_tick);
        // heartbeat_led_status.illuminated = !heartbeat_led_status.illuminated;

        // if(heartbeat_led_event) {
        //     heartbeat_led_event = 0;
        //     LOG_INF("Heartbeat LED toggled time = %lld ns", 2*(heartbeat_led_status.time2 - heartbeat_led_status.time1));
        // }

        // current_tick = k_uptime_ticks();
        // heartbeat_led_status.time1 = heartbeat_led_status.time2;
        // heartbeat_led_status.time2 = k_ticks_to_ns_near64(current_tick);
        // heartbeat_led_status.illuminated = !heartbeat_led_status.illuminated;

        // if(heartbeat_led_event) {
        //     heartbeat_led_event = 0;
        //     LOG_INF("Heartbeat LED toggled time = %lld ns", 2*(heartbeat_led_status.time2 - heartbeat_led_status.time1));
        // }
    } 
}


static const struct smf_state machine_states[];

enum machine_state { INIT, DEFAULT_SETUP, AWAKE, SLEEP, ERROR, ADC_READ, ADC_BLINK, IDLE };

struct s_object {
    /* This must be first */
    struct smf_ctx ctx;
    int last_action_led_toggle_freq ;
    int64_t last_action_led_toggle_time ;
    int sleep_button_pressed ;
    int timer_remaining ;
    // int heartbeat_led_event ;
    int action_led_event ;
    int error;
    int adc_read_value ;
    int adc_read_mv ;
    int return_value ;

    int32_t adc_on_time_ms ;
    int32_t adc_off_time_ms ;

} s_obj;


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

    // associate callback with GPIO pin
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
    s_obj.sleep_button_pressed =0;
    s_obj.timer_remaining =0;
    // int heartbeat_led_event ;
    s_obj.action_led_event =0;
    s_obj.error = 0;

    iv_pump_led_status.illuminated = 1;
    buzzer_led_status.illuminated = 0;

    gpio_pin_set_dt(&error_led, 0); // ensure error LED is off

    smf_set_state(SMF_CTX(&s_obj), &machine_states[IDLE]);
}

static void default_entry(void *o)
{
    LOG_INF("In default setup entry state.");

    // configure_buttons();
    s_obj.last_action_led_toggle_freq = LED_BLINK_FREQ_HZ;
    gpio_pin_set_dt(&iv_pump_led, iv_pump_led_status.illuminated);
    iv_pump_led_status.illuminated = iv_pump_led_status.illuminated;

    gpio_pin_set_dt(&buzzer_led, !iv_pump_led_status.illuminated);
    buzzer_led_status.illuminated = !iv_pump_led_status.illuminated;

    // k_timer_start(&timer_action, K_MSEC(1000 / (2 * s_obj.last_action_led_toggle_freq)), K_MSEC(1000 / (2 * s_obj.last_action_led_toggle_freq)));
}

static void default_run(void *o)
{
    LOG_INF("In default setup state.");

    smf_set_state(SMF_CTX(&s_obj), &machine_states[IDLE]);
}

static void awake_entry(void *o)
{
    LOG_INF("In awake entry state.");

    configure_buttons();
    gpio_pin_set_dt(&iv_pump_led, iv_pump_led_status.illuminated);
    gpio_pin_set_dt(&buzzer_led, buzzer_led_status.illuminated);
    k_timer_start(&timer_action, K_TICKS(s_obj.timer_remaining), K_MSEC(1000 / (2 * s_obj.last_action_led_toggle_freq)));

}

static void awake_run(void *o)
{
    LOG_INF("In awake run state.");
    uint32_t event = k_event_wait(&button_events, 0xF, true, K_FOREVER);

    if(event & SLEEP_BUTTON_EVENT){
        smf_set_state(SMF_CTX(&s_obj), &machine_states[SLEEP]);
        return;
    }

    if(event & RESET_BUTTON_EVENT){
        smf_set_state(SMF_CTX(&s_obj), &machine_states[DEFAULT_SETUP]);
        return;
    }

    if(event & FREQ_UP_BUTTON_EVENT) {
        LOG_INF("frequency increase/up button pressed in awake run state."); 
        increase_action_led_blink_frequency();
        if(s_obj.last_action_led_toggle_freq > ACTION_BUTTON_MAX_THRESHOLD_HZ){
            smf_set_state(SMF_CTX(&s_obj), &machine_states[ERROR]);
            return;
        } else {                    
            k_timer_start(&timer_action, K_MSEC(1000 / (2 * s_obj.last_action_led_toggle_freq)), K_MSEC(1000 / (2 * s_obj.last_action_led_toggle_freq)));
        }
    }

    if(event & FREQ_DOWN_BUTTON_EVENT) {
        LOG_INF("frequency decrease/down button pressed in awake run state."); 
        decrease_action_led_blink_frequency();
        if(s_obj.last_action_led_toggle_freq < ACTION_BUTTON_MIN_THRESHOLD_HZ){
            smf_set_state(SMF_CTX(&s_obj), &machine_states[ERROR]);
            return;
        }else {
            k_timer_start(&timer_action, K_MSEC(1000 / (2 * s_obj.last_action_led_toggle_freq)), K_MSEC(1000 / (2 * s_obj.last_action_led_toggle_freq)));
        }
    }
}

static void awake_exit(void *o)
{
    LOG_INF("In awake exit state.");
    k_timer_stop(&timer_action);
    gpio_pin_set_dt(&iv_pump_led, 0);
    gpio_pin_set_dt(&buzzer_led, 0);

    gpio_pin_interrupt_configure_dt(&freq_up_button, GPIO_INT_DISABLE);
    gpio_pin_interrupt_configure_dt(&freq_down_button, GPIO_INT_DISABLE);
}

static void sleep_run(void *o)
{
    LOG_INF("In sleep run state.");
    s_obj.timer_remaining = k_timer_remaining_get(&timer_action);
    LOG_INF("Timer remaining ticks: %d", s_obj.timer_remaining);

    uint32_t event = k_event_wait(&button_events, SLEEP_BUTTON_EVENT | RESET_BUTTON_EVENT, true, K_FOREVER);

    if (event & SLEEP_BUTTON_EVENT) {
        LOG_INF("sleep button pressed again, waking up.");
        smf_set_state(SMF_CTX(&s_obj), &machine_states[AWAKE]);
    } 

    if(event & RESET_BUTTON_EVENT) {
        LOG_INF("reset button pressed in sleep state."); 
        smf_set_state(SMF_CTX(&s_obj), &machine_states[DEFAULT_SETUP]);
    } 
}

static void sleep_exit(void *o)
{
    LOG_INF("In sleep exit state.");
    configure_buttons();
    // s_obj.error = gpio_pin_interrupt_configure_dt(&freq_up_button, GPIO_INT_EDGE_TO_ACTIVE); 
    // if (s_obj.error < 0) {
    //     LOG_ERR("Cannot attach callback to sw1.");
    // }

    // s_obj.error = gpio_pin_interrupt_configure_dt(&freq_down_button, GPIO_INT_EDGE_TO_ACTIVE); 
    // if (s_obj.error < 0) {
    //     LOG_ERR("Cannot attach callback to sw2.");
    // }
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
    s_obj.timer_remaining = k_timer_remaining_get(&timer_action);
    k_timer_stop(&timer_action);

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
    gpio_pin_set_dt(&buzzer_led, 0);
}

static void idle_run(void *o)
{
    LOG_INF("In idle run state.");
    uint32_t event = k_event_wait(&button_events, 0xF, true, K_FOREVER);

    if(event & SLEEP_BUTTON_EVENT){
        smf_set_state(SMF_CTX(&s_obj), &machine_states[ADC_READ]);
        return;
    }
    if(event & RESET_BUTTON_EVENT){
        smf_set_state(SMF_CTX(&s_obj), &machine_states[DEFAULT_SETUP]);
        return;
    }
    if(event & FREQ_UP_BUTTON_EVENT) {
        smf_set_state(SMF_CTX(&s_obj), &machine_states[SLEEP]);
        return;
    }
    if(event & FREQ_DOWN_BUTTON_EVENT) {
        smf_set_state(SMF_CTX(&s_obj), &machine_states[SLEEP]);
        return;
    }
}

static void adc_read_entry(void *o)
{
    LOG_INF("In ADC read entry state.");
    // init adc sequence

    LOG_INF("Measuring %s (channel %d)... ", adc_vadc.dev->name, adc_vadc.channel_id);

    (void)adc_sequence_init_dt(&adc_vadc, &sequence);
}

static void adc_read_run(void *o)
{   
    LOG_INF("In ADC read run state.");
    // aquition & check if valid
    s_obj.adc_read_value = adc_read(adc_vadc.dev, &sequence);

    if (s_obj.adc_read_value < 0) {
        LOG_ERR("Could not read (%d)", s_obj.error);
        smf_set_state(SMF_CTX(&s_obj), &machine_states[ERROR]);
    } else {
        LOG_DBG("Raw ADC Buffer: %d", s_obj.adc_read_value);
        smf_set_state(SMF_CTX(&s_obj), &machine_states[ADC_BLINK]);
    }
}

static void adc_read_exit(void *o)
{
    LOG_INF("In ADC read exit state.");
    // check voltage is in bounds, not go to blink if out of bouns and go to error , check valitiy 
    int32_t val_mv;  //note that adc_raw_to_millivolts_dt() takes a int32_t* to modify the value in place
    val_mv = s_obj.adc_read_value;  // val_mv is now the raw ADC value
    s_obj.adc_read_mv = adc_raw_to_millivolts_dt(&adc_vadc, &val_mv); // remember that the vadc struct containts all the DT parameters
    // val_mv now contains the ADC value in millivolts
    // add in some error checking
    if (s_obj.adc_read_mv < 0) {
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
    // enable thinsg only in state and disable 

    gpio_pin_interrupt_configure_dt(&freq_up_button, GPIO_INT_DISABLE);
    gpio_pin_interrupt_configure_dt(&freq_down_button, GPIO_INT_DISABLE);
    gpio_pin_interrupt_configure_dt(&sleep_button, GPIO_INT_DISABLE);
    gpio_pin_interrupt_configure_dt(&reset_button, GPIO_INT_DISABLE);

    /*
        disable button0 from being pressed while in this state
        set up timers for blinking
            one timer for blink rate - gpio toggle rate 
            one timer for 5 second duration (one shot timer)
                when finished, kills timer1 and turns off LED1
    */
}

static void adc_blink_run(void *o)
{
    LOG_INF("In ADC blink run state.");
    // blink
    /*
        Linearly map 0-3.0 V measured on the AIN0 input to an LED1 blink rate of 1-5 Hz 
            (e.g., 0 V -> 1 Hz; 3.0 V -> 5 Hz) with a duty cycle of 10%

        LED1 should remaining blinking for 5 seconds after the button has been pressed.
    */

    // if(s_obj.adc_read_mv >= 0 && s_obj.adc_read_mv <= 600){
    //     s_obj.last_action_led_toggle_freq = 1;
    // } else if(s_obj.adc_read_mv > 600 && s_obj.adc_read_mv <= 1200){
    //     s_obj.last_action_led_toggle_freq = 2;
    // } else if(s_obj.adc_read_mv > 1200 && s_obj.adc_read_mv <= 1800){
    //     s_obj.last_action_led_toggle_freq = 3;
    // } else if(s_obj.adc_read_mv > 1800 && s_obj.adc_read_mv <= 2400){
    //     s_obj.last_action_led_toggle_freq = 4;
    // } else if(s_obj.adc_read_mv > 2400 && s_obj.adc_read_mv <= 3000){
    //     s_obj.last_action_led_toggle_freq = 5;
    // }

    if(s_obj.adc_read_mv > 0 && s_obj.adc_read_mv < 3000) {
        s_obj.last_action_led_toggle_freq  = 1 + ((s_obj.adc_read_mv * (5 - 1)) / 3000); // map 0-3000 mV to 1-5 Hz
        LOG_INF("Mapped frequency (Hz): %d", s_obj.last_action_led_toggle_freq);

        s_obj.adc_on_time_ms = (1000 / s_obj.last_action_led_toggle_freq) / 10; // 10% duty cycle
        s_obj.adc_off_time_ms = (1000 / s_obj.last_action_led_toggle_freq) - s_obj.adc_on_time_ms;

        gpio_pin_set_dt(&iv_pump_led, 1);
        k_timer_start(&on_timer, K_MSEC(s_obj.adc_on_time_ms), K_NO_WAIT);
        // k_timer_start(&off_timer, K_MSEC(s_obj.adc_off_time_ms), K_NO_WAIT);

        k_timer_start(&duration_timer, K_MSEC(5000), K_NO_WAIT);

        // start timers for blinking
        // have a timer for the on time and then in the expiry handler, start the off time timer
        // and vice versa until 5 seconds is up
        // have another one shot timer for 5 seconds duration that will stop the blinking
        // when the one shot timer expires, stop the blinking timer and turn off LED1
        // can use k_timer_start() to start a timer with a specific duration
        // can use k_timer_stop() to stop a timer

        // put in a thread for 10% duty cycle blinking for 5 seconds
        // pass informartion related to the thread 

        // can be put in a state that just does the blinking for 5 seconds and then exits to idle state
       
        smf_set_state(SMF_CTX(&s_obj), &machine_states[IDLE]);
    } else {
        LOG_ERR("ADC reading out of bounds for blinking.");
        smf_set_state(SMF_CTX(&s_obj), &machine_states[ERROR]);
    }

    /*
    0V = 1 Hz
        10% duty cycle = on for 100 ms, off for 900 ms
    0.75V = 2 Hz
        10% duty cycle = on for 50 ms, off for 450 ms
    1.5V = 3 Hz
        10% duty cycle = on for 33 ms, off for 267 ms
    2.25V = 4 Hz
        10% duty cycle = on for 25 ms, off for 225 ms
    3V = 5 Hz -> 10% 
        10% duty cycle = on for 20 ms, off for 180 ms
    */
}

static void adc_blink_exit(void *o)
{
    LOG_INF("In ADC blink exit state.");
    // clean up things you just run 
    configure_buttons();
    gpio_pin_set_dt(&iv_pump_led, 0);
    gpio_pin_set_dt(&buzzer_led, 0);

}

/*
    if button0 is pressed (previous sleep button): 
        make a measurement using the AIN0 channel of the ADC
        Linearly map 0-3.0 V measured on the AIN0 input to an LED1 blink rate of 1-5 Hz 
            (e.g., 0 V -> 1 Hz; 3.0 V -> 5 Hz) with a duty cycle of 10%
        LED1 should remaining blinking for 5 seconds after the button has been pressed
        BUTTON0 should be disabled while the ADC measurement and LED1 blinking
        LED1 blinking should not occur in the SLEEP state; instead, if blinking happening when entering SLEEP, 
            it should stop until the next BUTTON0 press (not resume when waking)
        
    have timer1 for gpio toggle rate 
    have timer2 for 5 second duration of blinking (one shot timer)
        when finished, kills timer1 and turns off LED1

    error when reading -- go to error state

    tunrs on and goes to idle state, press 
    state for adc read, get value
        entry = init adc sequence
        run = aquition
        exit = check voltage is in bounds, not go to blink if out of bouns and go to error , check valitiy 
    state for timers blinking with frequency from adc read 
        entry = enable thinsg only in state and disable 
        run = blink
        exit = clean up things you just run 
        --> dont block for 5 seconds 
*/

// define states for state machine
static const struct smf_state machine_states[] = {
    [INIT] = SMF_CREATE_STATE(NULL, init_run, NULL, NULL, NULL),
    [DEFAULT_SETUP] = SMF_CREATE_STATE(default_entry, default_run, NULL, NULL, NULL),
    [IDLE] = SMF_CREATE_STATE(idle_entry, idle_run, NULL, NULL, NULL),
    [ADC_READ] = SMF_CREATE_STATE(adc_read_entry, adc_read_run, adc_read_exit, NULL, NULL),
    [ADC_BLINK] = SMF_CREATE_STATE(adc_blink_entry, adc_blink_run, adc_blink_exit, NULL, NULL), 
    [AWAKE] = SMF_CREATE_STATE(awake_entry, awake_run, awake_exit, NULL, NULL),
    [SLEEP] = SMF_CREATE_STATE(NULL, sleep_run, sleep_exit, NULL, NULL),
    [ERROR] = SMF_CREATE_STATE(error_entry, error_run, error_exit, NULL, NULL),
};

int main (void){
    int32_t ret;
    smf_set_initial(SMF_CTX(&s_obj), &machine_states[INIT]);

    while(1) {
        /* State machine terminates if a non-zero value is returned */
        int events = k_event_wait(&button_events, 0xF, true, K_MSEC(1000));
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
    // sleep_button_event = 1;  // conditional statement in main() can now do something based on the event detection
                             // we can also use actual system kernel event flags, but this is simpler (for now)
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



void increase_action_led_blink_frequency(void) {
    // increase the blink frequency of the action LEDs by 1 Hz, up to a max of 5 Hz
    s_obj.last_action_led_toggle_freq += FREQ_UP_INC_HZ;
    LOG_INF("Increased action LED blink frequency");
}

void decrease_action_led_blink_frequency(void) {
    // decrease the blink frequency of the action LEDs by 1 Hz, down to a min of 1 Hz
    s_obj.last_action_led_toggle_freq -= FREQ_DOWN_INC_HZ;
    LOG_INF("Decreased action LED blink frequency");
}


void action_timer_expiry_handler(struct k_timer *timer_action) {
    s_obj.action_led_event = 1;
    gpio_pin_toggle_dt(&iv_pump_led);
    int64_t current_tick2 = k_uptime_ticks();
    iv_pump_led_status.time1 = iv_pump_led_status.time2;
    iv_pump_led_status.time2 = k_ticks_to_ns_near64(current_tick2);
    iv_pump_led_status.illuminated = !iv_pump_led_status.illuminated;

    gpio_pin_toggle_dt(&buzzer_led);
    int64_t current_tick = k_uptime_ticks();
    buzzer_led_status.time1 = buzzer_led_status.time2;
    buzzer_led_status.time2 = k_ticks_to_ns_near64(current_tick);
    buzzer_led_status.illuminated = !buzzer_led_status.illuminated;

    if(s_obj.action_led_event) {
        s_obj.action_led_event = 0;
        LOG_INF("Action LEDs toggled time = %lld ns", 2*(iv_pump_led_status.time2 - iv_pump_led_status.time1));
        LOG_INF("Action LEDs frequency = %d Hz", s_obj.last_action_led_toggle_freq);
    }
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
}

void on_timer_interval_expiry_handler(struct k_timer *on_timer){
    gpio_pin_set_dt(&iv_pump_led, 0);
    k_timer_start(&off_timer, K_MSEC(s_obj.adc_off_time_ms), K_NO_WAIT);
}

void off_timer_interval_expiry_handler(struct k_timer *off_timer){
    gpio_pin_set_dt(&iv_pump_led, 1);
    k_timer_start(&on_timer, K_MSEC(s_obj.adc_on_time_ms), K_NO_WAIT);
}

void duration_timer_interval_expiry_handler(struct k_timer *duration_timer){
    k_timer_stop(&on_timer);
    k_timer_stop(&off_timer);
    gpio_pin_set_dt(&iv_pump_led, 0);
}



/*
    if button0 is pressed (previous sleep button): 
        make a measurement using the AIN0 channel of the ADC
        Linearly map 0-3.0 V measured on the AIN0 input to an LED1 blink rate of 1-5 Hz 
            (e.g., 0 V -> 1 Hz; 3.0 V -> 5 Hz) with a duty cycle of 10%
        LED1 should remaining blinking for 5 seconds after the button has been pressed
        BUTTON0 should be disabled while the ADC measurement and LED1 blinking
        LED1 blinking should not occur in the SLEEP state; instead, if blinking happening when entering SLEEP, 
            it should stop until the next BUTTON0 press (not resume when waking)
        
    have timer1 for gpio toggle rate 
    have timer2 for 5 second duration of blinking (one shot timer)
        when finished, kills timer1 and turns off LED1

    error when reading -- go to error state

    tunrs on and goes to idle state, press 
    state for adc read, get value
        entry = init adc sequence
        run = aquition
        exit = check voltage is in bounds, not go to blink if out of bouns and go to error , check valitiy 
    state for timers blinking with frequency from adc read 
        entry = enable thinsg only in state and disable 
        run = blink
        exit = clean up things you just run 
        --> dont block for 5 seconds 
*/