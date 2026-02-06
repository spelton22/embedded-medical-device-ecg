#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h> 
#include <zephyr/logging/log.h>  // needs CONFIG_LOG=y in your prj.conf
#include <zephyr/drivers/adc.h> // CONFIG_ADC=y
#include <zephyr/drivers/pwm.h> // CONFIG_PWM=y
#include <zephyr/smf.h> // CONFIG_SMF=y
#include <zephyr/drivers/sensor.h> // CONFIG_SENSOR=y
#include <math.h>
#include "calc_cycles.h"
#include "ble-lib.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

// DEFINE MACROS
#define HEARTBEAT_TOGGLE_INTERVAL_MS 500 // heartbeat LED toggles every 500 ms
#define BATTERY_TIMER_INTERVAL_MS 60000 // 1 minute
#define ERROR_LED_BLINK_INTERVAL_MS 500 // error LED blink interval
#define BUFFER_ARRAY_LEN 4000 // 20 samples/cycle × 20 cycles
#define SAMPLE_INTERVAL_US 2500 // 2.5 ms between samples (400 Hz sampling)
#define SAMPLE_TIME_SEC ((BUFFER_ARRAY_LEN *SAMPLE_INTERVAL_US)/1000000)  // total time to sample entire buffer
#define MEASUREMENT_DELAY_MS 1000 
#define HR_UPDATE_INTERVAL_MS 5000 // update heart rate every 5 seconds
#define LED_ON 1
#define LED_OFF 0
#define NOMINAL_BATTERY_VOLT_MV 3000
#define MINUTE_SEC 60
#define MINUTE_MSEC 60000
#define ECG_DUTY_CYCLE 0.25f // 25% duty cycle for

#define ERROR_INIT_MSG "Error occured during initialization" // any initialization
#define ERROR_CODE_ADC_MSG "Error occured during ADC measurement code" // any error with adc read, single or continuous
#define ERROR_CODE_PWM_MSG "Error occured during PWM measurment code" // pwm error
#define ERROR_CODE_TEMP_MSG "Error occured during temperature measurment code" // temp error

#define ADC_DT_SPEC_GET_BY_ALIAS(adc_alias)                    \
{                                                            \
    .dev = DEVICE_DT_GET(DT_PARENT(DT_ALIAS(adc_alias))),      \
    .channel_id = DT_REG_ADDR(DT_ALIAS(adc_alias)),            \
    ADC_CHANNEL_CFG_FROM_DT_NODE(DT_ALIAS(adc_alias))          \
} 

static enum adc_action diff_adc_sequence_callback(const struct device *dev, const struct adc_sequence *sequence, uint16_t sampling_index);

static const struct adc_dt_spec adc_vadc = ADC_DT_SPEC_GET_BY_ALIAS(vadc);
static const struct adc_dt_spec adc_vadc_diff = ADC_DT_SPEC_GET_BY_ALIAS(diffadc);


int16_t buf;
int16_t diff_adc_buffer[BUFFER_ARRAY_LEN];


struct adc_sequence sequence = {
    .buffer = &buf,
    .buffer_size = sizeof(buf), // bytes
};

struct adc_sequence_options options = {
    .extra_samplings = BUFFER_ARRAY_LEN - 1,  // -1 b/c first sample is already in the buffer
    .interval_us = SAMPLE_INTERVAL_US,  // 5ms between samples (200 Hz sampling)
    .callback = diff_adc_sequence_callback,  // called after each sample is collected
};

struct adc_sequence sequence_diff = {
    .options = &options,  // add the options to the sequence
    .buffer = &diff_adc_buffer,  // buf is now a pointer to the first index of an array
    .buffer_size = sizeof(diff_adc_buffer),  // need to specify the size of the buffer array in bytes
};

// PWM SETUP
static const struct pwm_dt_spec pwm = PWM_DT_SPEC_GET(DT_ALIAS(pwm));

// FUNCITON PROTOTYPES
void configure_buttons(void);
void process_ecg_buffer(const struct adc_dt_spec *adc_chan);
static void init_run(void *o);

static void idle_entry(void *o);
static void idle_run(void *o);
static void idle_exit(void *o);

static void error_entry(void *o);
static void error_run(void *o);
static void error_exit(void *o);

static void battery_measure_entry(void *o);
static void battery_measure_run(void *o);
static void battery_measure_exit(void *o);

static void reset_entry(void *o);
static void reset_run(void *o);
static void reset_exit(void *o);

static void battery_set_entry(void *o);
static void battery_set_run(void *o);
static void battery_set_exit(void *o);

static void temp_measure_entry(void *o);
static void temp_measure_run(void *o);
static void temp_measure_exit(void *o); 

static void heart_rate_measure_entry(void *o);
static void heart_rate_measure_run(void *o);
static void heart_rate_measure_exit(void *o);

// EVENT BIT ARRAYS
// uint8_t button_events_bit_array = 0x0;

K_EVENT_DEFINE(program_events);
#define IDLE_BUTTON_EVENT BIT(0)
#define TEMP_BUTTON_EVENT BIT(1)
#define HEART_RATE_BUTTON_EVENT BIT(2)
#define RESET_BUTTON_EVENT BIT(3)
#define BATTERY_TIMER_EVENT BIT(4)
#define ADC_DIFF_READ_EVENT BIT(5)
#define HR_UPDATE_EVENT BIT(6)

K_EVENT_DEFINE(error_events);
#define ERROR_INIT_EVENT BIT(0) // any initialization
#define ERROR_CODE_ADC_EVENT BIT(1) // any error with adc read, single or continuous
#define ERROR_CODE_PWM_EVENT BIT(2) // pwm error
#define ERROR_CODE_TEMP_EVENT BIT(3) // temp error


// TIMER FUNCTIONS
void hr_update_timer_interval_expiry_handler(struct k_timer *hr_update_timer);
K_TIMER_DEFINE(hr_update_timer, hr_update_timer_interval_expiry_handler, NULL);

void battery_measure_timer_interval_expiry_handler(struct k_timer *battery_timer);
K_TIMER_DEFINE(battery_measure_timer, battery_measure_timer_interval_expiry_handler, NULL);



void hr_on_timer_interval_expiry_handler(struct k_timer *hr_on_timer);
void hr_off_timer_interval_expiry_handler(struct k_timer *hr_off_timer);
K_TIMER_DEFINE(hr_on_timer, hr_on_timer_interval_expiry_handler, NULL);
K_TIMER_DEFINE(hr_off_timer, hr_off_timer_interval_expiry_handler, NULL);

// define globals and DT-based hardware structs
static const struct gpio_dt_spec idle_button = GPIO_DT_SPEC_GET(DT_ALIAS(idlebutton), gpios);
static const struct gpio_dt_spec temp_button = GPIO_DT_SPEC_GET(DT_ALIAS(tempbutton), gpios);
static const struct gpio_dt_spec heart_rate_button = GPIO_DT_SPEC_GET(DT_ALIAS(heartratebutton), gpios);
static const struct gpio_dt_spec reset_button = GPIO_DT_SPEC_GET(DT_ALIAS(resetbutton), gpios);
/*
    BUTTON0 = go to idle state from any measurement state (sleep_button)
    BUTTON1 = temperature measurement (freq_up_button)
    BUTTON2 = heart rate measurement (freq_down_button)
    BUTTON3 = reset device (reset_button)
*/

static const struct gpio_dt_spec heartbeat_led = GPIO_DT_SPEC_GET(DT_ALIAS(heartbeat), gpios);
static const struct gpio_dt_spec battery_led = GPIO_DT_SPEC_GET(DT_ALIAS(battery), gpios);
static const struct gpio_dt_spec heart_rate_led = GPIO_DT_SPEC_GET(DT_ALIAS(heartrate), gpios);
static const struct gpio_dt_spec error_led = GPIO_DT_SPEC_GET(DT_ALIAS(error), gpios);
/*
    LED0 = heartbeat_led
    LED1 = battery level indicator (iv_pump_led)
    LED2 = heart rate indicator (buzzer_led)
    LED3 = error indicator (error_led) (only on in error state)
*/

// define callback functions
// initialize GPIO Callback Structs

static struct gpio_callback idle_button_cb;  
static struct gpio_callback temp_button_cb;  
static struct gpio_callback heart_rate_button_cb;   
static struct gpio_callback reset_button_cb;

void idle_button_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
void temp_button_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
void heart_rate_button_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
void reset_button_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins);

// TEMPERATURE SENSOR SETUP
int read_temperature_sensor(const struct device *temp_sensor, float *temperature_degC);

// the microchip,mcp9808 cannot be accessed in the DT by alias
// instead, have to directly access the node name, with comma replaced by underscore
const struct device *const temp_sensor = DEVICE_DT_GET_ONE(jedec_jc_42_4_temp);

float temperature_degC;


// initialize GPIO Callback Structs
// struct led {
//   int64_t time1;  // int64_t b/c time functions in Zephyr use this type
//   int64_t time2;
//   bool illuminated; // state of the LED (on/off)
// };

// struct led heartbeat_led_status = {
//   .time1 = 0,
//   .time2 = 0,
//   .illuminated = false
// };

// struct led battery_led_status = {
//   .time1 = 0,
//   .time2 = 0,
//   .illuminated = false
// };

// struct led heart_rate_led_status = {
//   .time1 = 0,
//   .time2 = 0,
//   .illuminated = false
// };

// struct led error_led_status = {
//   .time1 = 0,
//   .time2 = 0,
//   .illuminated = false
// };

/*
    thread for heartbeat 

    - blink every 1 second with a 50% duty cycle in all states
*/

// HEARTBEAT_TOGGLE_INTERVAL_MS

int heartbeat_led_event = 0;

void heartbeat_thread(void *, void *, void *);

K_THREAD_DEFINE(heartbeat_thread_id, 1024, heartbeat_thread, NULL, NULL, NULL, 5, 0, 0);

void heartbeat_thread(void *a, void *b, void *c) {
  // k_msleep(2000);
  LOG_INF("Heartbeat thread started!");
  
  while (1) {
    // LOG_INF("Heartbeat LED ON");
    gpio_pin_set_dt(&heartbeat_led, LED_ON);
    k_msleep(HEARTBEAT_TOGGLE_INTERVAL_MS);
    // LOG_INF("Heartbeat LED OFF");
    gpio_pin_set_dt(&heartbeat_led, LED_OFF);
    k_msleep(HEARTBEAT_TOGGLE_INTERVAL_MS);
  } 
}

int error_led_event = 0;

void error_led_thread(void *, void *, void *);

K_THREAD_DEFINE(error_thread_id, 1024, error_led_thread, NULL, NULL, NULL, 5, 0, 0);

void error_led_thread(void *a, void *b, void *c) {
  // k_msleep(2000);
  LOG_INF("error thread started!");
  
  while (1) {
    LOG_INF("error LED ON");
    gpio_pin_set_dt(&heartbeat_led, LED_ON);
    gpio_pin_set_dt(&error_led, LED_ON);
    gpio_pin_set_dt(&battery_led, LED_ON);
    gpio_pin_set_dt(&heart_rate_led, LED_ON);

    k_msleep(HEARTBEAT_TOGGLE_INTERVAL_MS);

    LOG_INF("error LED OFF");
    gpio_pin_set_dt(&heartbeat_led, LED_OFF);
    gpio_pin_set_dt(&error_led, LED_OFF);
    gpio_pin_set_dt(&battery_led, LED_OFF);
    gpio_pin_set_dt(&heart_rate_led, LED_OFF);

    k_msleep(HEARTBEAT_TOGGLE_INTERVAL_MS);
  } 
}

// STATE MACHINE STATES
static const struct smf_state machine_states[];
enum machine_state { INIT, IDLE, ERROR, RESET, BATTERY_MEASURE, BATTERY_SET, TEMP_MEASURE, HEART_RATE_MEASURE /*, BLE_NOTIFY*/ };


struct s_object {
  struct smf_ctx ctx;
  int error;
  float adc_single_read_mv ;
  float heart_rate_bpm;
  float period_on;
  float period_off;
  float avg_bpm;
  float pwm_percent ;
  int init_error;
  int32_t raw_mv;
  int num_measurement;
  float total_bpm;
} s_obj;


/*
    notes:

    - funcitons should be short and do one thing
    - MACROS --> no hard coded values
    - use struct or organize global varibales and related data together
    - Use libraries for code that is self-contained
    - Use the LOGGING module to log errors, warnings, information and debug messages
*/


/*
    INIT STATE
    - all device intializstion

    <-- go to battery measure state to read battery voltage 
    
    done:
        - configure GPIO pins for buttons and LEDs
        - set up GPIO callbacks for buttons
        - initialize event object for button events
        - configure ADC channel for battery measurement
            - songle and continuous 
        - configure PWM for battery LED brightness control
*/
static void init_run(void *o)
{
    LOG_INF("init state");
    // LOG_INF("System Init State");

    k_event_init(&program_events);
    k_event_init(&error_events);
    // k_event_init(&timer_events);

    // check if interface is ready
    if (!device_is_ready(idle_button.port)) {
        s_obj.init_error = 1;
        // k_event_post(&error_events, ERROR_INIT_EVENT);
        // smf_set_state(SMF_CTX(&s_obj), &machine_states[ERROR]);
        LOG_INF("gpio0 interface not ready.");  // logging module output
        // return ;  // exit code that will exit main()
    }
    if (!device_is_ready(pwm.dev)) {
        s_obj.init_error = 1;
        // k_event_post(&error_events, ERROR_INIT_EVENT);
        // smf_set_state(SMF_CTX(&s_obj), &machine_states[ERROR]);
        LOG_INF("PWM device %s is not ready.", pwm.dev->name);
        // return ;
    }
    /* Check that the ADC interface is ready */
    if (!device_is_ready(adc_vadc.dev)) {
        s_obj.init_error = 1;
        // k_event_post(&error_events, ERROR_INIT_EVENT);
        // smf_set_state(SMF_CTX(&s_obj), &machine_states[ERROR]);
        LOG_INF("ADC controller device(s) not ready");
        // return ;
    }
    if (!device_is_ready(adc_vadc_diff.dev)) {
        s_obj.init_error = 1;
        // k_event_post(&error_events, ERROR_INIT_EVENT);
        // smf_set_state(SMF_CTX(&s_obj), &machine_states[ERROR]);
        LOG_INF("ADC controller device(s) not ready");
        // return ;
    }
    if (!device_is_ready(temp_sensor)) {
        s_obj.init_error = 1;
        // k_event_post(&error_events, ERROR_INIT_EVENT);
        // smf_set_state(SMF_CTX(&s_obj), &machine_states[ERROR]);
        LOG_INF("Temperature sensor %s is not ready", temp_sensor->name);
        // return ;
    }

    // configure GPIO pin
    s_obj.error = gpio_pin_configure_dt(&idle_button, GPIO_INPUT);
    if (s_obj.error < 0) {
        s_obj.init_error = 1;
        // k_event_post(&error_events, ERROR_INIT_EVENT);
        // smf_set_state(SMF_CTX(&s_obj), &machine_states[ERROR]);
        LOG_INF("Cannot configure sw0 pin.");
        // return ;
    }
    s_obj.error = gpio_pin_configure_dt(&reset_button, GPIO_INPUT);
    if (s_obj.error < 0) {
        s_obj.init_error = 1;
        // k_event_post(&error_events, ERROR_INIT_EVENT);
        // smf_set_state(SMF_CTX(&s_obj), &machine_states[ERROR]);
        LOG_INF("Cannot configure reset button.");
        // return ;
    }
    s_obj.error = gpio_pin_configure_dt(&temp_button, GPIO_INPUT);
    if (s_obj.error < 0) {
        s_obj.init_error = 1;
        // k_event_post(&error_events, ERROR_INIT_EVENT);
        // smf_set_state(SMF_CTX(&s_obj), &machine_states[ERROR]);
        LOG_INF("Cannot configure frequency up button.");
        // return ;
    }
    s_obj.error = gpio_pin_configure_dt(&heart_rate_button, GPIO_INPUT);
    if (s_obj.error < 0) {
        s_obj.init_error = 1;
        // k_event_post(&error_events, ERROR_INIT_EVENT);
        // smf_set_state(SMF_CTX(&s_obj), &machine_states[ERROR]);
        LOG_INF("Cannot configure frequency down button.");
        // return ;
    }

    // populate CB struct with information about the CB function and pin
    gpio_pin_interrupt_configure_dt(&idle_button, GPIO_INT_EDGE_TO_ACTIVE);
    gpio_init_callback(&idle_button_cb, idle_button_callback, BIT(idle_button.pin)); // associate callback with GPIO pin
    gpio_add_callback_dt(&idle_button, &idle_button_cb);

    gpio_pin_interrupt_configure_dt(&temp_button, GPIO_INT_EDGE_TO_ACTIVE);
    gpio_init_callback(&temp_button_cb, temp_button_callback, BIT(temp_button.pin)); // associate callback with GPIO pin
    gpio_add_callback_dt(&temp_button, &temp_button_cb);

    gpio_pin_interrupt_configure_dt(&heart_rate_button, GPIO_INT_EDGE_TO_ACTIVE);
    gpio_init_callback(&heart_rate_button_cb, heart_rate_button_callback, BIT(heart_rate_button.pin)); // associate callback with GPIO pin
    gpio_add_callback_dt(&heart_rate_button, &heart_rate_button_cb);

    gpio_pin_interrupt_configure_dt(&reset_button, GPIO_INT_EDGE_TO_ACTIVE);
    gpio_init_callback(&reset_button_cb, reset_button_callback, BIT(reset_button.pin)); // associate callback with GPIO pin
    gpio_add_callback_dt(&reset_button, &reset_button_cb);


    s_obj.error = gpio_pin_configure_dt(&heartbeat_led, GPIO_OUTPUT_ACTIVE);
    if (s_obj.error < 0) {
        s_obj.init_error = 1;
        // k_event_post(&error_events, ERROR_INIT_EVENT);
        // smf_set_state(SMF_CTX(&s_obj), &machine_states[ERROR]);
        LOG_INF("Cannot configure heartbeat LED.");
        // return ;
    }

    s_obj.error = gpio_pin_configure_dt(&battery_led, GPIO_OUTPUT_INACTIVE);
    if (s_obj.error < 0) {
        s_obj.init_error = 1;
        // k_event_post(&error_events, ERROR_INIT_EVENT);
        // smf_set_state(SMF_CTX(&s_obj), &machine_states[ERROR]);
        LOG_INF("Cannot configure iv pump LED.");
        // return ;
    }

    s_obj.error = gpio_pin_configure_dt(&heart_rate_led, GPIO_OUTPUT_INACTIVE);
    if (s_obj.error < 0) {
        s_obj.init_error = 1;
        // k_event_post(&error_events, ERROR_INIT_EVENT);
        // smf_set_state(SMF_CTX(&s_obj), &machine_states[ERROR]);
        LOG_INF("Cannot configure buzzer LED.");
        // return ;
    }

    s_obj.error = gpio_pin_configure_dt(&error_led, GPIO_OUTPUT_INACTIVE);
    if (s_obj.error < 0) {
        s_obj.init_error = 1;
        // k_event_post(&error_events, ERROR_INIT_EVENT);
        // smf_set_state(SMF_CTX(&s_obj), &machine_states[ERROR]);
        LOG_INF("Cannot configure buzzer LED.");
        // return ;
    }

    k_thread_suspend(error_thread_id); // suspend error LED thread initially


    if (s_obj.error < 0) {
        s_obj.init_error = 1;
        // k_event_post(&error_events, ERROR_INIT_EVENT);
        // smf_set_state(SMF_CTX(&s_obj), &machine_states[ERROR]);
        LOG_INF("Cannot configure error LED.");
        // return ;
    }

    /* Configure the ADC channel */
    s_obj.error = adc_channel_setup_dt(&adc_vadc);
    if (s_obj.error < 0) {
        s_obj.init_error = 1;
        // k_event_post(&error_events, ERROR_INIT_EVENT);
        // smf_set_state(SMF_CTX(&s_obj), &machine_states[ERROR]);
        LOG_INF("Could not setup ADC channel (%d)", s_obj.error);
        // return ;
    }

    s_obj.error = adc_channel_setup_dt(&adc_vadc_diff);
    if (s_obj.error < 0) {
        s_obj.init_error = 1;
        // k_event_post(&error_events, ERROR_INIT_EVENT);
        // smf_set_state(SMF_CTX(&s_obj), &machine_states[ERROR]);
        LOG_INF("Could not setup ADC channel (%d)", s_obj.error);
        // return ;
    }

    bluetooth_init(&bluetooth_callbacks, &remote_service_callbacks);
    
    LOG_INF("Initialization complete.");

    // smf_set_state(SMF_CTX(&s_obj), &machine_states[BATTERY_MEASURE]);

    if(s_obj.init_error == 1){
        s_obj.init_error = 0;
        k_event_post(&error_events, ERROR_INIT_EVENT);
        smf_set_state(SMF_CTX(&s_obj), &machine_states[ERROR]);
        // return ;
    } else {
        smf_set_state(SMF_CTX(&s_obj), &machine_states[BATTERY_MEASURE]);
    }
}

/*
    IDLE STATE

    ** no measurements taken 

    --> after reset state is complete 
    --> after done with temp measurement go to IDLE state
    --> after done with heart rate measurement go to IDLE state
    --> after battery measurement/set complete go to battery set state
    --> BUTTON0 press go to IDLE state from any measurement state 

    <-- after 1 minute timer expiry go to battery measurement state

    - after reset deivce go to IDLE state

    <-- BUTTON0 should return device to IDLE state from any measurement state (dont reset device)
    <-- BUTTON1 press = take a temp measurement
    <-- BUTTON2 press = take an instantaneous heart rate measurement

    battery measurments happens when ....
        1. device is first powered on -> straight from init state 
        2. every 1 minute thereafter , but only in IDLE state --> start tiner in idle entry state and turn off in exit state 
*/

static void idle_entry(void *o)
{
    LOG_INF("In idle entry state.");
    configure_buttons();
    gpio_pin_interrupt_configure_dt(&idle_button, GPIO_INT_DISABLE);
    k_timer_start(&battery_measure_timer, K_MSEC(BATTERY_TIMER_INTERVAL_MS), K_NO_WAIT); // start battery measurement time

}

static void idle_run(void *o)
{
    LOG_INF("In idle run state.");
    uint32_t event = k_event_wait(&program_events, BATTERY_TIMER_EVENT | TEMP_BUTTON_EVENT | HEART_RATE_BUTTON_EVENT | RESET_BUTTON_EVENT, true, K_FOREVER);
    // uint32_t timer_event = k_event_test(&program_events, BATTERY_TIMER_EVENT);

    if(event & BATTERY_TIMER_EVENT){
        LOG_INF("Battery measurement timer event received.");
        k_event_clear(&program_events, BATTERY_TIMER_EVENT);
        smf_set_state(SMF_CTX(&s_obj), &machine_states[BATTERY_MEASURE]);
        return;
    }
    if(event & TEMP_BUTTON_EVENT){
        LOG_INF("TEMP BUTTON PRESSED");
        k_event_clear(&program_events, TEMP_BUTTON_EVENT);
        smf_set_state(SMF_CTX(&s_obj), &machine_states[TEMP_MEASURE]);
        return;
    }
    if(event & HEART_RATE_BUTTON_EVENT){
        LOG_INF("HEART RATE BUTTON PRESSED");
        k_event_clear(&program_events, HEART_RATE_BUTTON_EVENT);
        smf_set_state(SMF_CTX(&s_obj), &machine_states[HEART_RATE_MEASURE]); // previously ADC_DIFF_READ
        return;
    }
    if(event & RESET_BUTTON_EVENT){
        LOG_INF("RESET BUTTON PRESSED");
        k_event_clear(&program_events, RESET_BUTTON_EVENT);
        smf_set_state(SMF_CTX(&s_obj), &machine_states[RESET]);
        return;
    }
}

static void idle_exit(void *o)
{
    LOG_INF("In idle exit state.");
    // configure_buttons();
    k_timer_stop(&battery_measure_timer); // stop battery measurement timer
    gpio_pin_interrupt_configure_dt(&idle_button, GPIO_INT_EDGE_TO_ACTIVE);
}

/*
    ERROR STATE --> any error exit codes occured in any function

    --> error in any state should go to error state
        -> post error related event (bit array)

    <-- BUTTON3 used to reset device from error state (go to reset state)

    - all 4 leds should blink at 50% duty cycle , in phase with each other (function)
        - override heartbeat thread (suspend)

    - error code should specify the error condition that caused the device to enter the error state 

    - use a BLE notification to send the error code 
*/

static void error_entry(void *o)
{
    LOG_INF("In error entry state.");
    
    // Turn off PWM for battery LED first
    pwm_set_pulse_dt(&pwm, 0);
    
    // Reconfigure battery LED as GPIO output (it was controlled by PWM)
    gpio_pin_configure_dt(&battery_led, GPIO_OUTPUT_INACTIVE);
    
    // Configure buttons - disable all except reset button
    gpio_pin_interrupt_configure_dt(&idle_button, GPIO_INT_DISABLE);
    gpio_pin_interrupt_configure_dt(&temp_button, GPIO_INT_DISABLE);
    gpio_pin_interrupt_configure_dt(&heart_rate_button, GPIO_INT_DISABLE);
    gpio_pin_interrupt_configure_dt(&reset_button, GPIO_INT_EDGE_TO_ACTIVE);

    k_thread_suspend(heartbeat_thread_id);
    k_thread_resume(error_thread_id);

    k_timer_stop(&hr_on_timer);
    k_timer_stop(&hr_off_timer);
    gpio_pin_set_dt(&heart_rate_led, LED_OFF);
}

// NOT DONE YET BLE !!!!!!!!
static void error_run(void *o)
{
    LOG_INF("In error run state.");

    /*
        all 4 leds blink at 50% duty cycle , in phase with each other
            - suspend heartbeat thread
    */

    // error_led_status.illuminated = 1;

    // gpio_pin_set_dt(&heartbeat_led, error_led_status.illuminated);
    // gpio_pin_set_dt(&battery_led, error_led_status.illuminated);
    // gpio_pin_set_dt(&heart_rate_led, error_led_status.illuminated);
    // gpio_pin_set_dt(&error_led, error_led_status.illuminated);
    // k_timer_start(&on_timer, K_MSEC(ERROR_LED_BLINK_INTERVAL_MS), K_NO_WAIT);

   /*
        - log error message with logging module
        - use BLE notification to send error code

        need to handle specific error codes here ?
   */

   uint32_t error_code = k_event_test(&error_events, ERROR_INIT_EVENT | ERROR_CODE_ADC_EVENT | ERROR_CODE_PWM_EVENT | ERROR_CODE_TEMP_EVENT);
    if(error_code & ERROR_INIT_EVENT) {
        LOG_INF("Error Code: Initialization Error."); 
        bluetooth_set_error(ERROR_INIT_MSG);
        // send ble notification with error code
        k_event_clear(&error_events, ERROR_INIT_EVENT);
    }
    if(error_code & ERROR_CODE_ADC_EVENT) {
        LOG_INF("Error Code: ADC Error."); 
        bluetooth_set_error(ERROR_CODE_ADC_MSG);
        // send ble notification with error code
        k_event_clear(&error_events, ERROR_CODE_ADC_EVENT);
    }
    if(error_code & ERROR_CODE_PWM_EVENT) {
        LOG_INF("Error Code: PWM Error."); 
        bluetooth_set_error(ERROR_CODE_PWM_MSG);
        // send ble notification with error code
        k_event_clear(&error_events, ERROR_CODE_PWM_EVENT);
    }
    if(error_code & ERROR_CODE_TEMP_EVENT) {
        LOG_INF("Error Code: Temp Error ."); 
        bluetooth_set_error(ERROR_CODE_TEMP_MSG);
        // send ble notification with error code
        k_event_clear(&error_events, ERROR_CODE_TEMP_EVENT);
    }

    LOG_INF("Waiting for reset button press to exit error state.");
    uint32_t event = k_event_wait(&program_events, RESET_BUTTON_EVENT, true, K_FOREVER);
    LOG_INF("Event received in error_run: 0x%08x", event);
    if(event & RESET_BUTTON_EVENT) {
      LOG_INF("reset button pressed in error."); 
      smf_set_state(SMF_CTX(&s_obj), &machine_states[RESET]);
      k_event_clear(&program_events, RESET_BUTTON_EVENT);
      return;
    }
}

static void error_exit(void *o)
{
    LOG_INF("In error exit state.");
    // gpio_pin_set_dt(&error_led, 0);
    configure_buttons();
    k_thread_suspend(error_thread_id);

    gpio_pin_set_dt(&heartbeat_led, LED_OFF);
    gpio_pin_set_dt(&error_led, LED_OFF);
    gpio_pin_set_dt(&battery_led, LED_OFF);
    gpio_pin_set_dt(&heart_rate_led, LED_OFF);

    k_thread_resume(heartbeat_thread_id);

    // k_timer_stop(&on_timer);
    // k_timer_stop(&off_timer);

    // turn heartbeat thread back on (resume)
}

/*
    RESET STATE 

    --> BUTTON3 used to reset device from error state
    --> BUTTON3 used to reset device from any measurement state 

    <-- done with reset go to IDLE state

    - go to IDLE 

    what needs to be reset ....
        - all measurement values should be reset 
        - temperature 
        - blink rates 
        - battery led pwm value 

        -- anything stored to memory 
*/

static void reset_entry(void *o)
{
    LOG_INF("In reset entry state.");
    // reset all measurement values and variables here 

    gpio_pin_interrupt_configure_dt(&idle_button, GPIO_INT_DISABLE);
    gpio_pin_interrupt_configure_dt(&temp_button, GPIO_INT_DISABLE);
    gpio_pin_interrupt_configure_dt(&heart_rate_button, GPIO_INT_DISABLE);
    gpio_pin_interrupt_configure_dt(&reset_button, GPIO_INT_DISABLE);

    k_timer_stop(&hr_on_timer);
    k_timer_stop(&hr_off_timer);
    gpio_pin_set_dt(&heart_rate_led, LED_OFF);
}

static void reset_run(void *o)
{
    LOG_INF("In reset run state.");

    /*
        what needs to be reset ....
            - all measurement values should be reset 
            - temperature 
            - blink rates 
            - battery led pwm value 

            -- anything stored to memory 
    */

    s_obj.error = 0;
    s_obj.adc_single_read_mv = 0.0;
    s_obj.heart_rate_bpm = 0.0;
    s_obj.period_on = 0.0;
    s_obj.period_off = 0.0;
    s_obj.avg_bpm = 0.0;
    s_obj.pwm_percent = 0.0;
    s_obj.init_error = 0;
    s_obj.raw_mv = 0;
    s_obj.num_measurement = 0;
    s_obj.total_bpm = 0.0;

    // error_led_status.illuminated = 0;
    // heart_rate_led_status.illuminated = 0;
    // battery_led_status.illuminated = 0;
    // heartbeat_led_status.illuminated = 0;

    temperature_degC = 0;

    smf_set_state(SMF_CTX(&s_obj), &machine_states[IDLE]);
    // return;
}

static void reset_exit(void *o)
{
    LOG_INF("In reset exit state.");
    configure_buttons();
}

/*
    GET BATTERY MEASUREMENT STATE

    --> device first powered on -> from init state
    --> every 1 minute thereafter , but only in IDLE state
    <-- done with adc read go to battery set state
    <-- BUTTON0 should return device to IDLE state from any measurement state (dont reset device)
    <-- BUTTON3 used to reset device (go to reset state)
    <-- BUTTON1 press = take a temp measurement
    <-- BUTTON2 press = take an instantaneous heart rate measurement go to heart rate measurement state

    - measure the input voltage 0-3.0 V using AIN0
        - single adc read 

    - send a BLE notification with the battery level percentage after each measurement

    measurments happens when ....
        1. device is first powered on -> straight from init state 
        2. every 1 minute thereafter , but only in IDLE state --> start tiner in idle entry state and turn off in exit state 

    need:
        - adc single shot read 
        - wait for events 
*/

static void battery_measure_entry(void *o)
{
    LOG_INF("In battery measure entry state.");
    LOG_INF("Measuring %s (channel %d)... ", adc_vadc.dev->name, adc_vadc.channel_id);
    // configure_buttons();
    
    // configure adc 
    (void)adc_sequence_init_dt(&adc_vadc, &sequence);
}

static void battery_measure_run(void *o)
{
    LOG_INF("In battery measure run state.");
    // if(s_obj.init_error == 1){
    //     s_obj.init_error = 0;
    //     k_event_post(&error_events, ERROR_INIT_EVENT);
    //     smf_set_state(SMF_CTX(&s_obj), &machine_states[ERROR]);
    //     return ;
    // }else {

        int ret = adc_read(adc_vadc.dev, &sequence);
        if (ret < 0) {
            LOG_INF("ADC read failed with error code %d", ret);
            k_event_post(&error_events, ERROR_CODE_ADC_EVENT);
            smf_set_state(SMF_CTX(&s_obj), &machine_states[ERROR]);
            // return;
        }else {
            LOG_INF("Raw ADC Buffer: %d", buf);
        }

        int32_t val_mv;  //note that adc_raw_to_millivolts_dt() takes a int32_t* to modify the value in place
        val_mv = buf;  // val_mv is now the raw ADC value
        ret = adc_raw_to_millivolts_dt(&adc_vadc, &val_mv); // remember that the vadc struct containts all the DT parameters
        s_obj.raw_mv = val_mv;
        s_obj.adc_single_read_mv = (float)val_mv;
        if(ret < 0) {
            LOG_INF("adc_raw_to_millivolts_dt() failed with error code %d", ret);
            k_event_post(&error_events, ERROR_CODE_ADC_EVENT);
            smf_set_state(SMF_CTX(&s_obj), &machine_states[ERROR]);
            // return;
        } else {
            LOG_INF("Battery Voltage reading: %d mV", val_mv);
            smf_set_state(SMF_CTX(&s_obj), &machine_states[BATTERY_SET]);
            // return;
        }

        bluetooth_set_battery_level(val_mv);
    /*
            wait for event:
                - button0 press = go to IDLE state
                - button1 press = go to TEMP_MEASURE state
                - button2 press = go to HEART_RATE_MEASURE state
                - button3 press = go to RESET state
        */
        uint32_t event = k_event_test(&program_events, IDLE_BUTTON_EVENT | TEMP_BUTTON_EVENT | HEART_RATE_BUTTON_EVENT | RESET_BUTTON_EVENT);
        
        if(event & IDLE_BUTTON_EVENT){
            LOG_INF("IDLE BUTTON PRESSED");
            k_event_clear(&program_events, IDLE_BUTTON_EVENT);
            smf_set_state(SMF_CTX(&s_obj), &machine_states[IDLE]);
            // return;
        }
        if(event & TEMP_BUTTON_EVENT){
            LOG_INF("TEMP BUTTON PRESSED");
            k_event_clear(&program_events, TEMP_BUTTON_EVENT);
            smf_set_state(SMF_CTX(&s_obj), &machine_states[TEMP_MEASURE]);
            // return;
        }
        if(event & HEART_RATE_BUTTON_EVENT){
            LOG_INF("HEART RATE BUTTON PRESSED");
            k_event_clear(&program_events, HEART_RATE_BUTTON_EVENT);
            smf_set_state(SMF_CTX(&s_obj), &machine_states[HEART_RATE_MEASURE]); // previously ADC_DIFF_READ
            // return;
        }
        if(event & RESET_BUTTON_EVENT){
            LOG_INF("RESET BUTTON PRESSED");
            k_event_clear(&program_events, RESET_BUTTON_EVENT);
            smf_set_state(SMF_CTX(&s_obj), &machine_states[RESET]);
            // return;
        }
    // }

    /*
        single adc measure read from AIN0 (0-3.0V)
            - calculate battery percentage from voltage reading

        - send a BLE notification with the battery level percentage after each measurement
        - log battery level with logging module
        - go to BATTERY_SET state
   */

    // smf_set_state(SMF_CTX(&s_obj), &machine_states[BATTERY_SET]);
    // return;
}

static void battery_measure_exit(void *o)
{
    LOG_INF("In battery measure exit state.");
    // configure_buttons();
}

/*
    SET BATTERY LED STATE

    --> after battery measurement state

    <-- done setting battery led brightness go to IDLE state
    <-- BUTTON0 should return device to IDLE state from any measurement state (dont reset device)
    <-- BUTTON3 used to reset device (go to reset state)
    <-- BUTTON1 press = take a temp measurement
    <-- BUTTON2 press = take an instantaneous heart rate measurement go to heart rate measurement state
    
    - have the brightness of LED1 linearly modulated by the percentage of battery level 
        --> PWM map 0-100% PWM to 0-3.0V 

    need: 
        - pwm 
        - wait for events 
*/

static void battery_set_entry(void *o)
{
    LOG_INF("In battery set entry state.");
    // configure_buttons();

    // configure pwm?
}

static void battery_set_run(void *o)
{
    LOG_INF("In battery set run state.");

    /*
        set battery led brightness based on battery percentage from previous state
            - map 0-100% battery to 0-3.0V using PWM
    */

    if(s_obj.adc_single_read_mv >= 0.0f){
        LOG_INF("Setting battery LED brightness based on battery voltage reading of %f mV", (double)s_obj.adc_single_read_mv);
        s_obj.pwm_percent  = (float)s_obj.adc_single_read_mv / 3000.0f; // assuming 3.0V is 100%
        // s_obj.pwm_percent = (float) 100 * s_obj.raw_mv / NOMINAL_BATTERY_VOLT_MV;
        LOG_INF("Battery percentage: %f%%", (double)s_obj.pwm_percent  * 100.0);

        int err;
        err = pwm_set_pulse_dt(&pwm, pwm.period*s_obj.pwm_percent );

        if(err){
            LOG_INF("Error setting PWM pulse width: %d", err);
            k_event_post(&error_events, ERROR_CODE_PWM_EVENT);
            smf_set_state(SMF_CTX(&s_obj), &machine_states[ERROR]);
            // return;
        }else {
            smf_set_state(SMF_CTX(&s_obj), &machine_states[IDLE]);
        }
   }else {
        LOG_INF("Invalid battery voltage reading.");
        k_event_post(&error_events, ERROR_CODE_ADC_EVENT);
        smf_set_state(SMF_CTX(&s_obj), &machine_states[ERROR]);
        // return;
   }

    /*
        wait for event:
            - button0 press = go to IDLE state
            - button1 press = go to TEMP_MEASURE state
            - button2 press = go to HEART_RATE_MEASURE state
            - button3 press = go to RESET state
    */
    uint32_t event = k_event_test(&program_events, IDLE_BUTTON_EVENT | TEMP_BUTTON_EVENT | HEART_RATE_BUTTON_EVENT | RESET_BUTTON_EVENT);
    if(event & IDLE_BUTTON_EVENT){
        LOG_INF("IDLE BUTTON PRESSED");
        k_event_clear(&program_events, IDLE_BUTTON_EVENT);
        smf_set_state(SMF_CTX(&s_obj), &machine_states[IDLE]);
        // return;
    }
    if(event & TEMP_BUTTON_EVENT){
        LOG_INF("TEMP BUTTON PRESSED");
        k_event_clear(&program_events, TEMP_BUTTON_EVENT);
        smf_set_state(SMF_CTX(&s_obj), &machine_states[TEMP_MEASURE]);
        // return;
    }
    if(event & HEART_RATE_BUTTON_EVENT){
        LOG_INF("HEART RATE BUTTON PRESSED");
        k_event_clear(&program_events, HEART_RATE_BUTTON_EVENT);
        smf_set_state(SMF_CTX(&s_obj), &machine_states[HEART_RATE_MEASURE]); // previously ADC_DIFF_READ
        // return;
    }
    if(event & RESET_BUTTON_EVENT){
        LOG_INF("RESET BUTTON PRESSED");
        k_event_clear(&program_events, RESET_BUTTON_EVENT);
        smf_set_state(SMF_CTX(&s_obj), &machine_states[RESET]);
        // return;
    }

//    smf_set_state(SMF_CTX(&s_obj), &machine_states[IDLE]);
//    return;
}

static void battery_set_exit(void *o)
{
    LOG_INF("In battery set exit state.");
    // configure_buttons();
    // pwm_set_pulse_dt(&pwm, 0);
}

/*
    TAKE TEMP MEASUREMENT STATE (on BUTTON1 press)

    --> BUTTON1 press = take a temp measurement

    <-- done with temp measurement go to IDLE state
    <-- BUTTON0 should return device to IDLE state from any measurement state (dont reset device)
    <-- BUTTON3 used to reset device (go to reset state)
    <-- BUTTON2 press = take an instantaneous heart rate measurement go to heart rate measurement state
    
    - make a measurement for MCP9808 temp sensor 
    - write to LOG_INF()

    -- code to read temp value: https://mlp6.github.io/Embedded-Medical-Devices/slides/zephyr-serial-communication.html

    -- maybe make this a library like calc_cycles 

    - send a BLE notification with the temp measurement after each measurement

*/

static void temp_measure_entry(void *o)
{
    LOG_INF("In temp measure entry state.");
    // configure_buttons();
    // k_timer_start(&hr_on_timer, K_MSEC(s_obj.period_on), K_NO_WAIT);
}

static void temp_measure_run(void *o)
{
    LOG_INF("In temp measure run state.");

    int ret = read_temperature_sensor(temp_sensor, &temperature_degC);
    if (ret != 0) {
        LOG_INF("There was a problem reading the temperature sensor (%d)", ret);
        k_event_post(&error_events, ERROR_CODE_TEMP_EVENT);
        smf_set_state(SMF_CTX(&s_obj), &machine_states[ERROR]);
        // return ;
    } else {
        smf_set_state(SMF_CTX(&s_obj), &machine_states[IDLE]);
    }
    
    LOG_INF("Temperature: %f", (double)temperature_degC);
    bluetooth_set_temperature(temperature_degC);
    k_msleep(MEASUREMENT_DELAY_MS);

    uint32_t event = k_event_test(&program_events, IDLE_BUTTON_EVENT | HEART_RATE_BUTTON_EVENT | RESET_BUTTON_EVENT);

    if(event & IDLE_BUTTON_EVENT){
        LOG_INF("IDLE BUTTON PRESSED");
        k_event_clear(&program_events, IDLE_BUTTON_EVENT);
        smf_set_state(SMF_CTX(&s_obj), &machine_states[IDLE]);
        // return;
    }
    if(event & HEART_RATE_BUTTON_EVENT){
        LOG_INF("HEART RATE BUTTON PRESSED");
        k_event_clear(&program_events, HEART_RATE_BUTTON_EVENT);
        smf_set_state(SMF_CTX(&s_obj), &machine_states[HEART_RATE_MEASURE]); // previously ADC_DIFF_READ
        // return;
    }
    if(event & RESET_BUTTON_EVENT){
        LOG_INF("RESET BUTTON PRESSED");
        k_event_clear(&program_events, RESET_BUTTON_EVENT);
        smf_set_state(SMF_CTX(&s_obj), &machine_states[RESET]);
        // return;
    }

    // smf_set_state(SMF_CTX(&s_obj), &machine_states[IDLE]);
}

static void temp_measure_exit(void *o)
{
    LOG_INF("In temp measure exit state.");
    // configure_buttons();
    // k_timer_stop(&hr_on_timer);
    // k_timer_stop(&hr_off_timer);
    // gpio_pin_set_dt(&heart_rate_led, LED_OFF);

}

/*
    MEASURE HEARTRATE STATE (on BUTTON2 press)

    --> BUTTON2 press = take an instantaneous heart rate measurement

    <-- BUTTON0 should return device to IDLE state from any measurement state (dont reset device)
    <-- BUTTON3 used to reset device (go to reset state)
    <-- BUTTON1 press = take a temp measurement
    <-- BUTTON2 press = go to IDLE and stop measurment 

    - expected between 40-200 BPM using ECG signal (ranging frm -500 to 500 mV, bipolar) (from function generator)

    -- pressing BUTTON2 in ECG measurement will STOP the live ECG measurement 

    - blink LED2 with 25% duty cycle at the average heart rate calculated
        -- update as often as feels appropriate (1 sec / 5 sec ????)

    - have a bluetooth notification to send the average heart rate after calculation
        - after the measurements are complete/updated and data have been processed
    
    - BUTTON1 can be pressed during ECG measurement to take temp measurement 

    - BUTTON0 should return device to IDLE state from any measurement state (dont reset device)
    - BUTTON3 used to reset device (go to reset state)

    need:
        - adc continuous read 
        - calculate average heart rate from ECG signal ??????
        - wait for events
        - check to make sure value is in range (40-200 BPM) (-500 to 500 mV)
        - blink LED2 at average heart rate with 25% duty cycle
*/

static void heart_rate_measure_entry(void *o)
{
    s_obj.error = 0 ;
    LOG_INF("In heart rate measure entry state.");
    // configure_buttons();
    LOG_INF("Measuring %s (channel %d)... ", adc_vadc_diff.dev->name, adc_vadc_diff.channel_id);
    (void)adc_sequence_init_dt(&adc_vadc_diff, &sequence_diff);

    s_obj.error = adc_read_async(adc_vadc_diff.dev, &sequence_diff, NULL);
    if (s_obj.error  < 0) {
        LOG_INF("Could not read differential ADC (%d)", s_obj.error );
        smf_set_state(SMF_CTX(&s_obj), &machine_states[ERROR]);
    } else {
        LOG_INF("Differential ADC read successful.");
    }
    LOG_INF("Raw Differential ADC Buffer first value: %d", diff_adc_buffer[0]);
}

static void heart_rate_measure_run(void *o)
{
    LOG_INF("In heart rate measure run state.");

    /*
        use adc continuous read 

        make sure reading is in range (40-200 BPM) (-500 to 500 mV)

        - blink LED2 with 25% duty cycle at the average heart rate calculated
            -- update as often as feels appropriate (1 sec / 5 sec ????)
        - have a bluetooth notification to send the average heart rate after calculation
            - after the measurements are complete/updated and data have been processed
    */

    // int ret = adc_read_async(adc_vadc_diff.dev, &sequence_diff, NULL);
    // if (ret < 0) {
    //     LOG_INF("Could not read differential ADC (%d)", ret);
    //     smf_set_state(SMF_CTX(&s_obj), &machine_states[ERROR]);
    // } else {
    //     LOG_INF("Differential ADC read successful.");
    // }
    // LOG_INF("Raw Differential ADC Buffer first value: %d", diff_adc_buffer[0]);

    
    /*
        wait for event:
            - button0 press = go to IDLE state
            - button1 press = go to TEMP_MEASURE state
            - button2 press = go to IDLE state (stop measurement) leave 
            - button3 press = go to RESET state
    */
    uint32_t event = k_event_test(&program_events, HR_UPDATE_EVENT | ADC_DIFF_READ_EVENT | IDLE_BUTTON_EVENT | TEMP_BUTTON_EVENT | HEART_RATE_BUTTON_EVENT | RESET_BUTTON_EVENT);
    // uint32_t diff_event = k_event_test(&async_diff_event, ADC_DIFF_READ_EVENT);
    // uint32_t hr_event = k_event_test(&hr_update_event, HR_UPDATE_EVENT);
    if(event & HR_UPDATE_EVENT){
        LOG_INF("Heart rate update timer event received.");
        k_event_clear(&program_events, HR_UPDATE_EVENT);

        s_obj.error  = adc_read_async(adc_vadc_diff.dev, &sequence_diff, NULL);
        if (s_obj.error  < 0) {
            LOG_INF("Could not read differential ADC (%d)", s_obj.error );
            k_event_post(&error_events, ERROR_CODE_ADC_EVENT);
            smf_set_state(SMF_CTX(&s_obj), &machine_states[ERROR]);
        } else {
            LOG_INF("Differential ADC read successful.");
        }
        LOG_INF("Raw Differential ADC Buffer first value: %d", diff_adc_buffer[0]);
    }
    if(event & ADC_DIFF_READ_EVENT){
        LOG_INF("Asynchronous differential ADC read event received.");

        k_event_clear(&program_events, ADC_DIFF_READ_EVENT);
        // LOG_INF("Asynchronous differential ADC read event received.");
        if(s_obj.error == 1){
            LOG_INF("Error during asynchronous differential ADC read.");
            k_event_post(&error_events, ERROR_CODE_ADC_EVENT);
            smf_set_state(SMF_CTX(&s_obj), &machine_states[ERROR]);
            // return;
        }
        // process data here
        // process_ecg_buffer(&adc_vadc_diff);
        int cycles = calc_cycles(diff_adc_buffer, BUFFER_ARRAY_LEN);
        // LOG_INF("Calculated Cycles: %d", cycles);
        float bpm = (((float)cycles * (float)MINUTE_SEC)) / ((float)SAMPLE_TIME_SEC);
        // LOG_INF("sampling time (s): %d", SAMPLE_TIME_SEC);
        LOG_INF("BPM = %f", (double) bpm);

        s_obj.num_measurement++;
        s_obj.total_bpm += bpm;
        s_obj.avg_bpm = ((float)s_obj.total_bpm) / ((float)s_obj.num_measurement);


        LOG_INF("Average Heart Rate: %f BPM", (double)s_obj.avg_bpm);
        bluetooth_set_heartrate(s_obj.avg_bpm);

        float full_period =  (float)MINUTE_MSEC / (float)bpm;
        s_obj.period_on = full_period * (float)ECG_DUTY_CYCLE;
        s_obj.period_off = full_period - s_obj.period_on;

        LOG_INF("LED ON TIME = %f ms", (double)s_obj.period_on);
        LOG_INF("LED OFF TIME = %f ms", (double)s_obj.period_off);


        gpio_pin_set_dt(&heart_rate_led, LED_ON);
        k_timer_start(&hr_on_timer, K_MSEC(s_obj.period_on), K_NO_WAIT);

        k_timer_start(&hr_update_timer, K_MSEC(HR_UPDATE_INTERVAL_MS), K_NO_WAIT);
    }
    if(event & IDLE_BUTTON_EVENT){
        LOG_INF("IDLE BUTTON PRESSED");
        k_event_clear(&program_events, IDLE_BUTTON_EVENT);
        smf_set_state(SMF_CTX(&s_obj), &machine_states[IDLE]);
        // return;
    }
    if(event & TEMP_BUTTON_EVENT){
        LOG_INF("TEMP BUTTON PRESSED");
        k_event_clear(&program_events, TEMP_BUTTON_EVENT);
        smf_set_state(SMF_CTX(&s_obj), &machine_states[TEMP_MEASURE]);
        // return;
    }
    if(event & HEART_RATE_BUTTON_EVENT){
        LOG_INF("HEART RATE BUTTON PRESSED");
        k_event_clear(&program_events, HEART_RATE_BUTTON_EVENT);
        smf_set_state(SMF_CTX(&s_obj), &machine_states[IDLE]); 
        // return;
    }
    if(event & RESET_BUTTON_EVENT){
        LOG_INF("RESET BUTTON PRESSED");
        k_event_clear(&program_events, RESET_BUTTON_EVENT);
        smf_set_state(SMF_CTX(&s_obj), &machine_states[RESET]);
        // return;
    }
    
    // smf_set_state(SMF_CTX(&s_obj), &machine_states[IDLE]);
}

static void heart_rate_measure_exit(void *o)
{
    LOG_INF("In heart rate measure exit state.");
    // configure_buttons();
    // k_timer_stop(&hr_on_timer);
    // k_timer_stop(&hr_off_timer);
    k_timer_stop(&hr_update_timer);
    // gpio_pin_set_dt(&heart_rate_led, LED_OFF);
}

/*
    SEND BLUETOOTH NOTIFICATION STATE

    -- have bluetooth notifications after the measurements are completed / updated and data have been processed

    - use BLE service 
        - configure DIS (device information serviced) to report the device model as team name (fun name)
        - set the BAS (battery service) to report the battery level of the device 
        - set the heart rate service to report the average heart rate 

    - set up a custom service to ....
        - TEMPERATURE for I2C temperature sensor data in degrees celcius
        - ERROR CODE for the error code that caused the device to enter the ERROR state 

    - BUTTON0 should return device to IDLE state from any measurement state (dont reset device)
    - BUTTON3 used to reset device (go to reset state)
*/

// static void ble_notify_entry(void *o)
// {
//     LOG_INF("In BLE notify entry state.");
//     // configure_buttons();
// }

// static void ble_notify_run(void *o)
// {
//     LOG_INF("In BLE notify run state.");

//     // send bluetooth notification code here 

//     smf_set_state(SMF_CTX(&s_obj), &machine_states[IDLE]);
// }

// static void ble_notify_exit(void *o)
// {
//     LOG_INF("In BLE notify exit state.");
//     // configure_buttons();
// }


static const struct smf_state machine_states[] = {
    [INIT] = SMF_CREATE_STATE(NULL, init_run, NULL, NULL, NULL),
    [IDLE] = SMF_CREATE_STATE(idle_entry, idle_run, idle_exit, NULL, NULL),
    [ERROR] = SMF_CREATE_STATE(error_entry, error_run, error_exit, NULL, NULL),
    [RESET] = SMF_CREATE_STATE(reset_entry, reset_run, reset_exit, NULL, NULL),
    [BATTERY_MEASURE] = SMF_CREATE_STATE(battery_measure_entry, battery_measure_run, battery_measure_exit, NULL, NULL),
    [BATTERY_SET] = SMF_CREATE_STATE(battery_set_entry, battery_set_run, battery_set_exit, NULL, NULL),
    [TEMP_MEASURE] = SMF_CREATE_STATE(temp_measure_entry, temp_measure_run, temp_measure_exit, NULL, NULL),
    [HEART_RATE_MEASURE] = SMF_CREATE_STATE(heart_rate_measure_entry, heart_rate_measure_run, heart_rate_measure_exit, NULL, NULL),
    // [BLE_NOTIFY] = SMF_CREATE_STATE(ble_notify_entry, ble_notify_run, ble_notify_exit, NULL, NULL),
};


int main (void){
    LOG_INF("Starting State Machine Application");
    int32_t ret;
    smf_set_initial(SMF_CTX(&s_obj), &machine_states[INIT]);

    while(1) {
        /* State machine terminates if a non-zero value is returned */
        // int events = k_event_wait(&button_events, SLEEP_BUTTON_EVENT | FREQ_DOWN_BUTTON_EVENT | FREQ_DOWN_BUTTON_EVENT | RESET_BUTTON_EVENT, true, K_MSEC(1000));
        ret = smf_run_state(SMF_CTX(&s_obj));

        if (ret) {
            // to do -- init error 
            smf_set_terminate(SMF_CTX(&s_obj), ret);
            break;
        }
        k_msleep(1000);
    }
}

// define callback functions
void idle_button_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
  k_event_post(&program_events, IDLE_BUTTON_EVENT);
}

void temp_button_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
  k_event_post(&program_events, TEMP_BUTTON_EVENT);
}

void heart_rate_button_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
  k_event_post(&program_events, HEART_RATE_BUTTON_EVENT);
}

void reset_button_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
  LOG_INF("Reset button callback triggered!");
  k_event_post(&program_events, RESET_BUTTON_EVENT);
}

void configure_buttons(void) {
    s_obj.error = gpio_pin_interrupt_configure_dt(&idle_button, GPIO_INT_EDGE_TO_ACTIVE); 
    if (s_obj.error < 0) {
        LOG_INF("Cannot attach callback to sw1.");
    }

    s_obj.error = gpio_pin_interrupt_configure_dt(&heart_rate_button, GPIO_INT_EDGE_TO_ACTIVE); 
    if (s_obj.error < 0) {
        LOG_INF("Cannot attach callback to sw2.");
    }

    s_obj.error = gpio_pin_interrupt_configure_dt(&temp_button, GPIO_INT_EDGE_TO_ACTIVE); 
    if (s_obj.error < 0) {
        LOG_INF("Cannot attach callback to sw0.");
    }

    s_obj.error = gpio_pin_interrupt_configure_dt(&reset_button, GPIO_INT_EDGE_TO_ACTIVE); 
    if (s_obj.error < 0) {
        LOG_INF("Cannot attach callback to sw3.");
    }
    
    LOG_INF("finished configuring buttons!!");
}

void battery_measure_timer_interval_expiry_handler(struct k_timer *battery_timer) {
    LOG_INF("Battery measurement timer expired.");
    k_event_post(&program_events, BATTERY_TIMER_EVENT);
    // k_timer_start(&battery_measure_timer, K_MSEC(BATTERY_TIMER_INTERVAL_MS), K_NO_WAIT);
    // post an event to take battery measurement
    // can define a new event or reuse existing ones based on design
}


/*
void hr_update_work_handler(struct k_work *work) {
    int32_t val_mv = diff_adc_buffer[s_obj.samp_index];

    int ret = adc_raw_to_millivolts_dt(&adc_vadc_diff, &val_mv); // remember that the vadc struct containts all the DT parameters
    
    if( ret < 0){
        s_obj.error = 1;
    }

    s_obj.adc_single_read_mv = (float)val_mv;  

    // expected to range from 40-200 BPM, using an ECG signal (ranging from -500 - 500 mV, note this is bipolar) 
    float heart_rate = (s_obj.adc_single_read_mv + 500.0f) * (160.0f / 1000.0f) + 40.0f; // map -500 to 500 mV to 40 to 200 BPM
    
    // float bpm = 0.16f * s_obj.adc_single_read_mv + 120.0f;

    // set LED2 blink rate to heart rate with 25% duty cycle
    if(heart_rate < 40.0f){
        heart_rate = 40.0f;
    }
    if(heart_rate > 200.0f){
        heart_rate = 200.0f;
    }
    s_obj.heart_rate_bpm = heart_rate;
    // LOG_INF("Heart Rate: %f BPM", (double)s_obj.heart_rate_bpm);
    // int blink_interval_ms = (int)(60000.0f / s_obj.heart_rate_bpm); // calculate blink interval in ms
    s_obj.period_on = 0.25f * (60.0f / s_obj.heart_rate_bpm) * 1000.0f; // on time in ms
    s_obj.period_off = 0.75f * (60.0f / s_obj.heart_rate_bpm) * 1000.0f; // off time in ms
    //toggle on off of light at 25% duty cycle and heart rate

    // create on timer and off timer for heart rate led

    // in state when leave, make sure to stop timers
    
    
    k_msleep(1);
    
    s_obj.samp_index++;
    // LOG_INF("index=%d\n", s_obj.samp_index);
    if (s_obj.samp_index == BUFFER_ARRAY_LEN - 1) {
        s_obj.samp_index = 0;
    }
}
*/


static enum adc_action diff_adc_sequence_callback(const struct device *dev, const struct adc_sequence *sequence, uint16_t sampling_index){

    if(sampling_index == BUFFER_ARRAY_LEN-1){
        LOG_INF("adc diff event posted");
        k_event_post(&program_events, ADC_DIFF_READ_EVENT);
        LOG_INF("All samples collected in callback.");
        return ADC_ACTION_FINISH;
    } 
    return ADC_ACTION_CONTINUE;
}

void hr_update_timer_interval_expiry_handler(struct k_timer *hr_update_timer) {
    k_event_post(&program_events, HR_UPDATE_EVENT);
}


// void on_timer_interval_expiry_handler(struct k_timer *on_timer) {
//   LOG_INF("Error LED ON timer expired.");
//   gpio_pin_set_dt(&heartbeat_led, LED_OFF);
//   gpio_pin_set_dt(&battery_led, LED_OFF);
//   gpio_pin_set_dt(&heart_rate_led, LED_OFF);
//   gpio_pin_set_dt(&error_led, LED_OFF);

//   k_timer_start(&off_timer, K_MSEC(ERROR_LED_BLINK_INTERVAL_MS), K_NO_WAIT);
// }

// void off_timer_interval_expiry_handler(struct k_timer *off_timer) {
//   LOG_INF("Error LED OFF timer expired.");
//   gpio_pin_set_dt(&heartbeat_led, LED_ON);
//   gpio_pin_set_dt(&battery_led, LED_ON);
//   gpio_pin_set_dt(&heart_rate_led, LED_ON);
//   gpio_pin_set_dt(&error_led, LED_ON);

//   k_timer_start(&on_timer, K_MSEC(ERROR_LED_BLINK_INTERVAL_MS), K_NO_WAIT);
// }

void hr_on_timer_interval_expiry_handler(struct k_timer *hr_on_timer) {
    // LOG_INF("Heart Rate LED ON timer expired.");
    gpio_pin_set_dt(&heart_rate_led, LED_OFF);
    k_timer_start(&hr_off_timer, K_MSEC(s_obj.period_off), K_NO_WAIT);
}

void hr_off_timer_interval_expiry_handler(struct k_timer *hr_off_timer) {
    // LOG_INF("Heart Rate LED OFF timer expired.");
    gpio_pin_set_dt(&heart_rate_led, LED_ON);
    k_timer_start(&hr_on_timer, K_MSEC(s_obj.period_on), K_NO_WAIT);
}

int read_temperature_sensor(const struct device *temp_sensor, float *temperature_degC) {
    /*  Fetch-n-get temperature sensor data
        INPUTS:
            temp_sensor (const struct device *) - temperature sensor device
            temperature_degC (float *) - pointer to store temperature in degrees Celsius
        RETURNS:
            0 - success
            Otherwise, error code
    */
    struct sensor_value sensor_vals = {.val1 = 0, .val2 = 0};
    int err = sensor_sample_fetch(temp_sensor);
    if (err != 0) {
        LOG_INF("Temperature sensor fetch(): %d", err);
        return err;
    }
    else {
        // sensor channels: https://docs.zephyrproject.org/latest/doxygen/html/group__sensor__interface.html#gaaa1b502bc029b10d7b23b0a25ef4e934

        err = sensor_channel_get(temp_sensor, SENSOR_CHAN_AMBIENT_TEMP, &sensor_vals);
        if (err != 0) {
            LOG_INF("Temperature sensor get(): %d", err);
            return err;
        }
    }
        
    // data returned in kPa
    *temperature_degC = sensor_value_to_float(&sensor_vals);
    LOG_INF("Temperature (deg C): %f", (double)*temperature_degC);
    return 0;
}


/*
    NOTES

    - k_event_test() to see if an event occured 

    - Use timers, kernel events, work queues, threads and any other Zephyr RTOS features as needed to implement the above functionality
*/


/*
    for timer for taking battery, should start timer in idle then when the timer ends, post an event and handle the event in idle state 
    
    if not in idle state then battery measuremnt is not taken 
*/