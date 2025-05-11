#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <stdint.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <driver/gpio.h>

#include "vl53l0x_esp32.h"
#include "mma845x.h"
#include "motor_control.h"
#include "bdc_motor.h"

#include <esp_wifi.h>
#include <nvs_flash.h>
#include <lwip/sockets.h>
#include <lwip/err.h>
#include <lwip/sys.h>
#include <esp_system.h>
#include <esp_event.h>
#include <esp_mac.h>
#include <esp_timer.h>

#include <math.h>
#include <freertos/queue.h>
#include <string.h>
#include "driver/ledc.h"
#include "esp_err.h"
#include <stdint.h>
#include "driver/adc.h"

// #include "soc/rtc_wdt.h"

#define ONBOARD_LED 38

#define DIST_R90 15
#define DIST_RF  16
#define DIST_R   7
#define DIST_L   6
#define DIST_LF  5
#define DIST_L90 4
#define DIST_F   21
#define DIST_F_INT_PIN 14

#define LINE_RIGHT 18  //RIGHT
#define LINE_LEFT  8  //LEFT 

#define MODE_BUTTON 2
#define START_STOP_MODULE 1
#define BLACK_FIELD 1

#define LONG_PRESS_DELAY 1000
#define DOUBLE_PRESS_DELAY 3000
// #define SINGLE_RANGING
#define CONTINUOUS_RANGING
#define MAX_RANGE 600
#define MIN_RANGE 10

// motors setup
#define BDC_LEDC_TIMER_RESOLUTION_HZ 1000000   // 1MHz, 1 tick = 0.1us
#define BDC_LEDC_FREQ_HZ             25000     // 25KHz PWM
#define BDC_LEDC_DUTY_TICK_MAX       (BDC_LEDC_TIMER_RESOLUTION_HZ / BDC_LEDC_FREQ_HZ)  // max value we can set for the duty cycle, in ticks
#define BDC_LEDC_GPIO_1A             9         // Motor 1 A control pin
#define BDC_LEDC_GPIO_1B             10        // Motor 1 B control pin
#define BDC_LEDC_GPIO_2A             11        // Motor 2 A control pin
#define BDC_LEDC_GPIO_2B             12        // Motor 2 B control pin
#define MAX_DUTY                     1023

#define PATROL_SPEED 55
#define ATTACK_SPEED 60
#define TURN_SPEED 50

#define PIN_ADC1_CHANNEL_4 13  // GPIO13 mapped to ADC1_CHANNEL_4 //LEFT
#define PIN_ADC1_CHANNEL_7 18  // GPIO18 mapped to ADC1_CHANNEL_7 //RIGHT

typedef struct {
    ledc_channel_t channel_a;
    ledc_channel_t channel_b;
    ledc_mode_t speed_mode;
} motor_t;

// convert percentage to duty
static inline uint32_t speed_to_duty(uint8_t speed_percent) {
    if(speed_percent > 100) speed_percent = 100;
    return (MAX_DUTY * speed_percent) / 100;
}

esp_err_t motor_forward(motor_t* motor, uint8_t speed_percent) {
    uint32_t duty = speed_to_duty(speed_percent);
    ESP_ERROR_CHECK(ledc_set_duty(motor->speed_mode, motor->channel_a, 0));
    ESP_ERROR_CHECK(ledc_update_duty(motor->speed_mode, motor->channel_a));

    ESP_ERROR_CHECK(ledc_set_duty(motor->speed_mode, motor->channel_b, duty));
    ESP_ERROR_CHECK(ledc_update_duty(motor->speed_mode, motor->channel_b));
    return ESP_OK;
}

esp_err_t motor_reverse(motor_t* motor, uint8_t speed_percent) {
    uint32_t duty = speed_to_duty(speed_percent);
    ESP_ERROR_CHECK(ledc_set_duty(motor->speed_mode, motor->channel_a, duty));
    ESP_ERROR_CHECK(ledc_update_duty(motor->speed_mode, motor->channel_a));

    ESP_ERROR_CHECK(ledc_set_duty(motor->speed_mode, motor->channel_b, 0));
    ESP_ERROR_CHECK(ledc_update_duty(motor->speed_mode, motor->channel_b));
    return ESP_OK;
}

esp_err_t motor_brake(motor_t* motor) {
    ESP_ERROR_CHECK(ledc_set_duty(motor->speed_mode, motor->channel_a, MAX_DUTY));
    ESP_ERROR_CHECK(ledc_update_duty(motor->speed_mode, motor->channel_a));

    ESP_ERROR_CHECK(ledc_set_duty(motor->speed_mode, motor->channel_b, MAX_DUTY));
    ESP_ERROR_CHECK(ledc_update_duty(motor->speed_mode, motor->channel_b));
    return ESP_OK;
}

esp_err_t motor_coast(motor_t* motor) {
    ESP_ERROR_CHECK(ledc_set_duty(motor->speed_mode, motor->channel_a, 0));
    ESP_ERROR_CHECK(ledc_update_duty(motor->speed_mode, motor->channel_a));

    ESP_ERROR_CHECK(ledc_set_duty(motor->speed_mode, motor->channel_b, 0));
    ESP_ERROR_CHECK(ledc_update_duty(motor->speed_mode, motor->channel_b));
    return ESP_OK;
}

// LEDC configuration for Motor 1
ledc_channel_config_t ledc_channel1A = {
    .gpio_num = BDC_LEDC_GPIO_1A,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = LEDC_CHANNEL_0,
    .timer_sel = LEDC_TIMER_0,
    .duty = 0,   // Initial duty cycle (0% power)
    .hpoint = 0
};

ledc_channel_config_t ledc_channel1B = {
    .gpio_num = BDC_LEDC_GPIO_1B,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = LEDC_CHANNEL_1,
    .timer_sel = LEDC_TIMER_0,
    .duty = 0,   // Initial duty cycle (0% power)
    .hpoint = 0
};

// LEDC configuration for Motor 2
ledc_channel_config_t ledc_channel2A = {
    .gpio_num = BDC_LEDC_GPIO_2A,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = LEDC_CHANNEL_2,
    .timer_sel = LEDC_TIMER_1,
    .duty = 0,   // Initial duty cycle (0% power)
    .hpoint = 0
};

ledc_channel_config_t ledc_channel2B = {
    .gpio_num = BDC_LEDC_GPIO_2B,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = LEDC_CHANNEL_3,
    .timer_sel = LEDC_TIMER_1,
    .duty = 0,   // Initial duty cycle (0% power)
    .hpoint = 0
};

// Motor 1 and Motor 2 structures
motor_t motor1 = {
    .channel_a = LEDC_CHANNEL_0,
    .channel_b = LEDC_CHANNEL_1,
    .speed_mode = LEDC_LOW_SPEED_MODE
};

motor_t motor2 = {
    .channel_a = LEDC_CHANNEL_2,
    .channel_b = LEDC_CHANNEL_3,
    .speed_mode = LEDC_LOW_SPEED_MODE
};

typedef enum {
    PATROL,
    CENTER,
    ATTACK,
    RETREAT,
    ROTATE,
    FORWARD,
    IDLE
}  state_t;

// #define ACTIVE_ACCEL
// #define ACTIVE_DEBUG
// #define SENSORS_DEBUG
// #define ACTIVE_DEBUG_DISTANCE_1
// #define ACTIVE_DEBUG_DISTANCE_2
// #define ACTIVE_DEBUG_DISTANCE_3
// #define ACTIVE_DEBUG_LINE
// #define SAFE_MODE
#define USE_START_STOP_MODULE

esp_timer_handle_t motorControlTimer = NULL;
esp_timer_handle_t sensorReadingTimer = NULL;
esp_timer_handle_t retreatTimer = NULL;
esp_timer_handle_t wingMotorTimer = NULL;

int mode = 0;
int patrol_state = 0;
int digital_distace = 25;
static const char *MAIN_TAG = "main";

typedef struct
{
    float move_forward_duty_cycle;
    float move_backward_duty_cycle;
    float turn_duty_cycle;
    int move_delay;
    int turn_delay;
} PatrolParams;

// SENSOR ORDER(ROBOT VIEW): FR F FL R90 R45 L45 L90
VL53L0X_Error status = VL53L0X_ERROR_NONE;
uint8_t dist_sensor_xshut = DIST_F;
uint16_t dist_sensor_data;
VL53L0X_Dev_t front_sensor;
VL53L0X_RangingMeasurementData_t measurement;

//R90 RF R L LF L90
int dist_sensor_pins[6] = {DIST_R90, DIST_RF, DIST_R, DIST_L, DIST_LF, DIST_L90};
int sensor_data[6] = {0, 0, 0, 0, 0, 0};

SemaphoreHandle_t sensor_data_mutex = NULL;
SemaphoreHandle_t is_running_mutex = NULL;
SemaphoreHandle_t data_mutex = NULL;
SemaphoreHandle_t line_sensor_mutex = NULL;
TaskHandle_t sensor_task_handle = NULL;

int output;
int direction = 0;
state_t state = CENTER;

int line_right, line_left, line_back;

int output_MOTOR_CALLBACK = 0;
state_t state_MOTOR_CALLBACK = PATROL;

bool retreating = false;
bool is_running = false;
bool is_wing_running = true;

void sensor_task(void* arg);

void IRAM_ATTR vl53l0x_isr_handler(void* arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(sensor_task_handle, &xHigherPriorityTaskWoken);
    if(xHigherPriorityTaskWoken) portYIELD_FROM_ISR();
}

void setup_vl53l0x_interrupt_pin()
{
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_NEGEDGE,  // Active low trigger
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << DIST_F_INT_PIN),
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE
    };
    gpio_config(&io_conf);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(DIST_F_INT_PIN, vl53l0x_isr_handler, NULL);
}

// calibration min/max values
int adc1_min = 4095, adc1_max = 0;
int adc2_min = 4095, adc2_max = 0;

void calibrate_sensors() {
    ESP_LOGI("mama", "Calibrating...");
    for(int i = 0; i < 200; i++) {
        int val1 = adc1_get_raw(ADC1_CHANNEL_4);
        int val2 = adc1_get_raw(ADC1_CHANNEL_7);

        if(val1 < adc1_min) adc1_min = val1;
        if(val1 > adc1_max) adc1_max = val1;

        if(val2 < adc2_min) adc2_min = val2;
        if(val2 > adc2_max) adc2_max = val2;

        vTaskDelay(pdMS_TO_TICKS(20));
    }
    ESP_LOGI("mama", "Calibration complete.");
    ESP_LOGI("mama", "ADC1: min=%d max=%d | ADC2: min=%d max=%d", adc1_min, adc1_max, adc2_min, adc2_max);
}

int normalize(int value, int min, int max) {
    if (max == min) return 0;  // Prevent division by zero

    // Use floating-point to avoid integer truncation
    float normalized = ((float)(value - min) / (max - min)) * 100;

    // Ensure normalized value is within the 0-100 range
    if (normalized < 0) normalized = 0;
    if (normalized > 100) normalized = 100;

    return (int)normalized;  // Cast back to int
}



//TODO: set lower timing budget
static int setup()
{
    int return_value = 1;
    gpio_reset_pin(ONBOARD_LED);
    gpio_set_direction(ONBOARD_LED, GPIO_MODE_OUTPUT);
    gpio_set_level(ONBOARD_LED, 1);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    gpio_set_level(ONBOARD_LED, 0);

    #ifdef ACTIVE_DEBUG
    ESP_LOGI(MAIN_TAG, "Create DC motors");
    #endif

    // Configure LEDC Timer 0 for Motor 1
    ledc_timer_config_t ledc_timer1 = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_10_BIT,  // 13-bit resolution for the duty cycle (0-8191)
        .freq_hz = BDC_LEDC_FREQ_HZ,           // 25KHz PWM frequency
        .clk_cfg = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer1));

    // Configure LEDC Timer 1 for Motor 2
    ledc_timer_config_t ledc_timer2 = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_1,
        .duty_resolution = LEDC_TIMER_10_BIT,  // 13-bit resolution for the duty cycle (0-8191)
        .freq_hz = BDC_LEDC_FREQ_HZ,           // 25KHz PWM frequency
        .clk_cfg = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer2));

    // Configure LEDC channels for Motor 1 and Motor 2
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel1A));
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel1B));
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel2A));
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel2B));

    //mode button
    gpio_reset_pin(MODE_BUTTON);
    gpio_set_direction(MODE_BUTTON, GPIO_MODE_INPUT);

    //start-stop modulex
    gpio_reset_pin(START_STOP_MODULE);
    gpio_set_direction(START_STOP_MODULE, GPIO_MODE_INPUT);

    // // line sensors (analog)
    // adc1_config_width(ADC_WIDTH);
    // adc1_config_channel_atten(ADC1_CHANNEL, ADC_ATTEN);
    // adc2_config_channel_atten(ADC2_CHANNEL, ADC_ATTEN);

    //dist sensors
    #ifdef ACTIVE_DEBUG
    ESP_LOGI(MAIN_TAG, "XHSUT resetting");
    #endif

    #ifdef ACTIVE_DEBUG
    ESP_LOGI(MAIN_TAG, "I2C initializing");
    #endif
    i2c_master_init();

    #ifdef ACTIVE_DEBUG
    ESP_LOGI(MAIN_TAG, "XSHUTs initializing");
    #endif

    init_xshuts(&dist_sensor_xshut, 1);
    set_all_xshut_states(&dist_sensor_xshut, true, 1);
    vTaskDelay(25 / portTICK_PERIOD_MS);
    set_all_xshut_states(&dist_sensor_xshut, false, 1);
    vTaskDelay(25 / portTICK_PERIOD_MS);

    status = init_dist_sensor(&front_sensor, dist_sensor_xshut, DIST_F, VL53L0X_HIGH_SPEED);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    if(status != VL53L0X_ERROR_NONE)
    {
        #ifdef ACTIVE_DEBUG
        ESP_LOGE(MAIN_TAG, "Dist sensor initialization failed");
        ESP_LOGE(MAIN_TAG, "Error: %d", status);
        #endif
        return 0;
    }

    // 1. set device mode BEFORE threshold config
    status = VL53L0X_SetDeviceMode(&front_sensor, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
    if(status != VL53L0X_ERROR_NONE)
    {
        #ifdef ACTIVE_DEBUG
        ESP_LOGE(MAIN_TAG, "Dist sensor device mode failed");
        ESP_LOGE(MAIN_TAG, "Error: %d", status);
        #endif
        return 0;
    }

    // 2. set thresholds (REQUIRES device mode set first)
    uint32_t low_mm = 10;
    uint32_t high_mm = 150;
    status = VL53L0X_SetInterruptThresholds(&front_sensor, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING, low_mm, high_mm);
    if(status != VL53L0X_ERROR_NONE)
    {
        #ifdef ACTIVE_DEBUG
        ESP_LOGE(MAIN_TAG, "Setting thresholds failed");
        ESP_LOGE(MAIN_TAG, "Error: %d", status);
        #endif
        return 0;
    }

    // 3. set interrupt config (must follow threshold config)
    status = VL53L0X_SetGpioConfig(
        &front_sensor,                             // Sensor device handle
        0,                                         // GPIO Pin (only Pin 0 is accepted)
        VL53L0X_DEVICEMODE_CONTINUOUS_RANGING,     // Device mode (e.g., continuous ranging mode)
        VL53L0X_GPIOFUNCTIONALITY_NEW_MEASURE_READY, // GPIO functionality (interrupt on threshold crossing)
        VL53L0X_INTERRUPTPOLARITY_LOW              // Interrupt polarity (active low)
    );    


    if(status != VL53L0X_ERROR_NONE)
    {
        #ifdef ACTIVE_DEBUG
        ESP_LOGE(MAIN_TAG, "Dist sensor GPIO config failed");
        ESP_LOGE(MAIN_TAG, "Error: %d", status);
        #endif
        return 0;
    }
    
    // // 4. setup ESP32 pin for interrupt (your custom function)
    // data_mutex = xSemaphoreCreateMutex();
    // xTaskCreate(sensor_task, "sensor_task", 4096, NULL, 1, &sensor_task_handle);
    // setup_vl53l0x_interrupt_pin();

    // // 5. start ranging
    // status = VL53L0X_StartMeasurement(&front_sensor);
    // if(status != VL53L0X_ERROR_NONE)
    // {
    //     #ifdef ACTIVE_DEBUG
    //     ESP_LOGE(MAIN_TAG, "Start measurement failed");
    //     ESP_LOGE(MAIN_TAG, "Error: %d", status);
    //     #endif
    //     return 0;
    // }

    #ifdef ACTIVE_DEBUG
    ESP_LOGI(MAIN_TAG, "Dist sensors initializing");
    #endif
    gpio_reset_pin(DIST_R90);
    gpio_set_direction(DIST_R90, GPIO_MODE_INPUT);
    gpio_reset_pin(DIST_RF);
    gpio_set_direction(DIST_RF, GPIO_MODE_INPUT);
    gpio_reset_pin(DIST_R);
    gpio_set_direction(DIST_R, GPIO_MODE_INPUT);
    gpio_reset_pin(DIST_L);
    gpio_set_direction(DIST_L, GPIO_MODE_INPUT);
    gpio_reset_pin(DIST_LF);
    gpio_set_direction(DIST_LF, GPIO_MODE_INPUT);
    gpio_reset_pin(DIST_L90);
    gpio_set_direction(DIST_L90, GPIO_MODE_INPUT);

    //line sensors
    #ifdef ACTIVE_DEBUG
    ESP_LOGI(MAIN_TAG, "Line sensors initializing");
    #endif
    // Configure ADC width and resolution
    adc1_config_width(ADC_WIDTH_BIT_12);  // 12-bit resolution (0-4095)
    
    // Configure ADC1 channels (GPIO13 and GPIO18)
    // adc1_config_channel_atten(ADC1_CHANNEL_4, ADC_ATTEN_DB_11);  // 0 dB attenuation for GPIO13
    // adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_DB_11);  // 0 dB attenuation for GPIO18
    // calibrate_sensors();
    return return_value;

}
  
static void mode_select()
{
    int press_time = 0;
    int double_press = 0;
    mode = 0;

    //detect long press
    while(!double_press)
    {
        if(gpio_get_level(MODE_BUTTON) == 0)
        {
            //detect long press
            while(gpio_get_level(MODE_BUTTON) == 0)
            {
                vTaskDelay(100 / portTICK_PERIOD_MS);
                press_time += 100;
            }

            if(press_time >= LONG_PRESS_DELAY)
            {
                mode++;
                press_time = 0;
                #ifdef ACTIVE_DEBUG
                ESP_LOGI(MAIN_TAG, "Mode %d selected", mode);
                #endif

                //blink LED for mode selection
                for(int i = 0; i < mode; i++)
                {
                    gpio_set_level(ONBOARD_LED, 1);
                    vTaskDelay(250 / portTICK_PERIOD_MS);
                    gpio_set_level(ONBOARD_LED, 0);
                    vTaskDelay(250 / portTICK_PERIOD_MS);
                }
            }

            else
            {
                //detect double press
                while(press_time < DOUBLE_PRESS_DELAY)
                {
                    if(gpio_get_level(MODE_BUTTON) == 0)
                    {
                        while(gpio_get_level(MODE_BUTTON) == 0)
                        {
                            vTaskDelay(100 / portTICK_PERIOD_MS);
                            press_time += 100;
                        }
                        double_press = 1;
                        break;
                    }
                    vTaskDelay(100 / portTICK_PERIOD_MS);
                    press_time += 100;
                }
            }
        }
    }

    //turn on LED after mode selection
    gpio_set_level(ONBOARD_LED, 1);
}

void blink_led(int cnt, int delay)
{
    for(int i = 0; i < cnt; i++)
    {
        gpio_set_level(ONBOARD_LED, 1);
        vTaskDelay(delay / portTICK_PERIOD_MS);
        gpio_set_level(ONBOARD_LED, 0);
        vTaskDelay(delay / portTICK_PERIOD_MS);
    }
}

void sensor_task(void* arg)
{
    sensor_task_handle = xTaskGetCurrentTaskHandle();

    for(;;)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // block until ISR notifies

        VL53L0X_RangingMeasurementData_t data;
        VL53L0X_GetRangingMeasurementData(&front_sensor, &data);

        // Check if the measurement is valid
        if(data.RangeStatus == 0)
        {
            // Check if the distance is within a valid range (e.g., 30 mm to 1000 mm)
            if (data.RangeMilliMeter >= 5 && data.RangeMilliMeter <= 125)
            {
                #ifdef ACTIVE_DEBUG
                ESP_LOGI(MAIN_TAG, "Distance: %d mm", data.RangeMilliMeter);
                #endif
                xSemaphoreTake(data_mutex, portMAX_DELAY);
                memcpy(&measurement, &data, sizeof(data));
                xSemaphoreGive(data_mutex);
            }
            else
            {
                // Handle out-of-bounds distance (optional logging or action)
                // ESP_LOGW(MAIN_TAG, "Invalid distance: %d mm", data.RangeMilliMeter);
                data.RangeMilliMeter = 0; // Set to 0 or some other default value
                xSemaphoreTake(data_mutex, portMAX_DELAY);
                memcpy(&measurement, &data, sizeof(data));
                xSemaphoreGive(data_mutex);
            }
        }
        else
        {
            // Handle invalid measurement (optional logging or action)
            ESP_LOGW(MAIN_TAG, "Range status not valid: %d", data.RangeStatus);
            xSemaphoreTake(data_mutex, portMAX_DELAY);
            memcpy(&measurement, &data, sizeof(data));
            xSemaphoreGive(data_mutex);
        }

        // Clear the interrupt (important to avoid re-triggering)
        VL53L0X_ClearInterruptMask(&front_sensor, 0);
    }
}



void gpio_update_task(void *arg)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(100); // 10ms interval

    while(1)
    {
        // int adc_value1 = adc1_get_raw(ADC1_CHANNEL); // 18
        // int adc_value2 = 0;
        // esp_err_t ret = adc2_get_raw(ADC2_CHANNEL, ADC_WIDTH, &adc_value2); // 8
        int adc_value_1 = adc1_get_raw(ADC1_CHANNEL_4); //line left
        int adc_value_2 = adc1_get_raw(ADC1_CHANNEL_7); // line right

        // // Log the ADC values (range: 0-4095 for 12-bit resolution)
        // ESP_LOGI("ADC", "ADC1 (GPIO18) value: %d", adc_value1);
        // if (ret == ESP_OK) {
        //     ESP_LOGI("ADC", "ADC2 (GPIO8) value: %d", adc_value2);
        // } else {
        //     ESP_LOGE("ADC", "Failed to read ADC2 (GPIO8)");
        // }

        if (xSemaphoreTake(sensor_data_mutex, (TickType_t)10) == pdTRUE)
        {
            for(int i = 0; i < 6; i++)
            {
                sensor_data[i] = gpio_get_level(dist_sensor_pins[i]);
            }
            xSemaphoreGive(sensor_data_mutex);
        }

        if (xSemaphoreTake(is_running_mutex, (TickType_t)10) == pdTRUE)
        {
            is_running = gpio_get_level(START_STOP_MODULE);
            xSemaphoreGive(is_running_mutex);
        }

        // if(xSemaphoreTake(line_sensor_mutex, (TickType_t)10) == pdTRUE){
        //     line_left = adc_value_1;
        //     line_right = adc_value_2;
        //     xSemaphoreGive(line_sensor_mutex);
        // }

        #ifdef ACTIVE_DEBUG
        ESP_LOGI(MAIN_TAG, "Line right: %d", line_right);
        ESP_LOGI(MAIN_TAG, "Line left: %d", line_left);
        ESP_LOGI(MAIN_TAG, "Distance sensors: %d %d %d %d %d %d", 
            sensor_data[0], sensor_data[1], sensor_data[2], 
            sensor_data[3], sensor_data[4], sensor_data[5]);
        #endif

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

static const char *TAG = "STATE_MACHINE";

void state_machine_task(void *arg)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10); // 10ms interval
    int local_is_running = 1;
    int sensor_snapshot[6] = {0};
    VL53L0X_RangingMeasurementData_t local_measurement;
    int local_line_right = 0;
    int local_line_left = 0;
    int curr_direction = 1; // -1 left, 1 right
    int burst_duration = 10;
    int last_burst_time = 0;
    int burst_stop_duration = 10;
    
    // Define static variables outside your loop or make them persistent inside the function
    static int oscillation_direction = 1; // 1 = left, -1 = right
    static TickType_t last_switch_time = 0;
    static TickType_t last_lost_target_time = 0;
    const TickType_t oscillation_interval = pdMS_TO_TICKS(30);     // 1s left/right
    const TickType_t stabilize_duration = pdMS_TO_TICKS(15);       // 2s forward-only after losing target
    const TickType_t patrol_cycle_duration = pdMS_TO_TICKS(1000);  
    const TickType_t patrol_move_duration = pdMS_TO_TICKS(100);   
    TickType_t patrol_last_switch_time = 0;
    bool patrol_moving = true;

    while(local_is_running)
    {
        // update is_running locally
        if(xSemaphoreTake(is_running_mutex, pdMS_TO_TICKS(5)) == pdTRUE)
        {
            local_is_running = is_running;
            xSemaphoreGive(is_running_mutex);
        }

        // take snapshot of 6 sensors
        if(xSemaphoreTake(sensor_data_mutex, pdMS_TO_TICKS(5)) == pdTRUE)
        {
            memcpy(sensor_snapshot, sensor_data, sizeof(sensor_snapshot));
            xSemaphoreGive(sensor_data_mutex);
        }

        // take snapshot of distance sensor
        if(xSemaphoreTake(data_mutex, pdMS_TO_TICKS(5)) == pdTRUE)
        {
            memcpy(&local_measurement, &measurement, sizeof(local_measurement));
            xSemaphoreGive(data_mutex);
        }

        // if(xSemaphoreTake(line_sensor_mutex, pdMS_TO_TICKS(5)) == pdTRUE)
        // {
        //     local_line_left = line_left;
        //     local_line_right = line_right;
        //     xSemaphoreGive(line_sensor_mutex);
        // 

        int left_sum = sensor_snapshot[3] + sensor_snapshot[4] + sensor_snapshot[5];
        int right_sum = sensor_snapshot[0] + sensor_snapshot[1] + sensor_snapshot[2];
        int mid_sum = sensor_snapshot[2] + sensor_snapshot[3];
        int any_detected = left_sum + right_sum > 0;


        int left_weight = sensor_snapshot[3] * 1 + sensor_snapshot[4] * 3 + sensor_snapshot[5] * 4;
        int right_weight = sensor_snapshot[0] * 4 + sensor_snapshot[1] * 3 + sensor_snapshot[2] * 1;


        #ifdef ACTIVE_DEBUG
            ESP_LOGI(TAG, "Sensors: [%d %d %d %d %d %d %d] | Left: %d, Right: %d, Mid: %d, Any: %d, Line: %d",
                     sensor_snapshot[0], sensor_snapshot[1], sensor_snapshot[2],
                     sensor_snapshot[3], sensor_snapshot[4], sensor_snapshot[5], sensor_snapshot[6],
                     left_sum, right_sum, mid_sum, any_detected, line_detected);
        #endif

        switch(mode)
        {
            // MODE 1 - SIMPLE FORWARD PATROL WITH CENTERING AND ATTACK OF OPPONENT
            case 1:
                #ifdef ACTIVE_DEBUG
                    ESP_LOGI(TAG, "Mode 1 | State: %d", state);
                #endif
                switch(state)
                {
                    case PATROL:
                        #ifdef ACTIVE_DEBUG
                            ESP_LOGI(TAG, "PATROL: forward(70, 70)");
                        #endif
                        motor_forward(&motor1, PATROL_SPEED);
                        motor_forward(&motor2, PATROL_SPEED);
                        if(any_detected)
                        {
                            #ifdef ACTIVE_DEBUG
                                ESP_LOGI(TAG, "PATROL: target detected, switching to CENTER");
                            #endif
                            state = CENTER;
                        }
                        break;

                    case CENTER:
                        if(mid_sum > 0)
                        {
                            // Mid sensors detected the target, stop and check for front sensors
                            state = ATTACK;
                            break;
                        }
                        else if(left_sum > right_sum)
                        {
                            // Rotate right if left sensors have a higher sum
                            int boost = right_weight; // Adjust turning speed based on weight
                            #ifdef ACTIVE_DEBUG
                                ESP_LOGI(TAG, "CENTER: rotating right (boost %d)", boost);
                            #endif
                            motor_forward(&motor1, TURN_SPEED + boost);
                            motor_reverse(&motor2, TURN_SPEED + boost);
                            curr_direction = 1;
                        }
                        else if(right_sum > left_sum)
                        {
                            // Rotate left if right sensors have a higher sum
                            int boost = left_weight; // Adjust turning speed based on weight
                            #ifdef ACTIVE_DEBUG
                                ESP_LOGI(TAG, "CENTER: rotating left (boost %d)", boost);
                            #endif
                            motor_forward(&motor2, TURN_SPEED + boost);
                            motor_reverse(&motor1, TURN_SPEED + boost);
                            curr_direction = -1;
                        }
                        else
                            state = PATROL;
                        break;


                    case ATTACK:
                        #ifdef ACTIVE_DEBUG
                            ESP_LOGI(TAG, "ATTACK: forward(90, 90)");
                        #endif
                        // if(local_measurement.RangeMilliMeter > 0 && local_measurement.RangeStatus == 0)
                        // {
                        //     motor_forward(&motor1, 100);
                        //     motor_forward(&motor2, 100);
                        //     ESP_LOGI(TAG, "FULL SPEED AHEAD");
                        // }
                        // else 
                        if (sensor_snapshot[2] && sensor_snapshot[3])
                        {
                            motor_forward(&motor1, 100);
                            motor_forward(&motor2, 100);
                        }
                        else
                        {
                            #ifdef ACTIVE_DEBUG
                                ESP_LOGI(TAG, "ATTACK: lost target, switching to PATROL");
                            #endif
                            last_lost_target_time = xLastWakeTime;
                            state = PATROL;
                        }
                        break;

                    default:
                        #ifdef ACTIVE_DEBUG
                            ESP_LOGW(TAG, "Mode 2: Unknown state %d", state);
                        #endif
                        break;
                }
                break;

            case 2:
                #ifdef ACTIVE_DEBUG
                ESP_LOGI(TAG, "Mode 2 | State: %d", state);
                #endif
                switch(state)
                {
                    case PATROL:
                        if(any_detected)
                        {
                            #ifdef ACTIVE_DEBUG
                                ESP_LOGI(TAG, "PATROL: target detected, switching to CENTER");
                            #endif
                            state = CENTER;
                        }
                        else if(xLastWakeTime - last_lost_target_time < stabilize_duration)
                        {
                            // Stabilize by going straight
                            #ifdef ACTIVE_DEBUG
                                ESP_LOGI(TAG, "PATROL: stabilizing forward motion");
                            #endif
                            motor_forward(&motor1, PATROL_SPEED);
                            motor_forward(&motor2, PATROL_SPEED);
                        }
                        else
                        {
                            // Oscillate left and right
                            if(xLastWakeTime - last_switch_time >= oscillation_interval)
                            {
                                oscillation_direction *= -1;
                                last_switch_time = xLastWakeTime;
                            }

                            if(oscillation_direction == 1)
                            {
                                #ifdef ACTIVE_DEBUG
                                    ESP_LOGI(TAG, "PATROL: oscillating left");
                                #endif
                                motor_forward(&motor1, PATROL_SPEED + 20);
                                motor_reverse(&motor2, PATROL_SPEED - 10);
                            }
                            else
                            {
                                #ifdef ACTIVE_DEBUG
                                    ESP_LOGI(TAG, "PATROL: oscillating right");
                                #endif
                                motor_reverse(&motor1, PATROL_SPEED - 10);
                                motor_forward(&motor2, PATROL_SPEED + 20);
                            }
                        }
                        break;

                    case CENTER:
                        if(mid_sum > 0)
                        {
                            // Mid sensors detected the target, stop and check for front sensors
                            state = ATTACK;
                            break;
                        }
                        else if(left_sum > right_sum)
                        {
                            // Rotate right if left sensors have a higher sum
                            int boost = right_weight; // Adjust turning speed based on weight
                            #ifdef ACTIVE_DEBUG
                                ESP_LOGI(TAG, "CENTER: rotating right (boost %d)", boost);
                            #endif
                            motor_forward(&motor1, TURN_SPEED + boost);
                            motor_reverse(&motor2, TURN_SPEED + boost);
                            curr_direction = 1;
                        }
                        else if(right_sum > left_sum)
                        {
                            // Rotate left if right sensors have a higher sum
                            int boost = left_weight; // Adjust turning speed based on weight
                            #ifdef ACTIVE_DEBUG
                                ESP_LOGI(TAG, "CENTER: rotating left (boost %d)", boost);
                            #endif
                            motor_forward(&motor2, TURN_SPEED + boost);
                            motor_reverse(&motor1, TURN_SPEED + boost);
                            curr_direction = -1;
                        }
                        else
                            state = PATROL;
                        break;


                    case ATTACK:
                        #ifdef ACTIVE_DEBUG
                            ESP_LOGI(TAG, "ATTACK: forward(90, 90)");
                        #endif
                        // if(local_measurement.RangeMilliMeter > 0 && local_measurement.RangeStatus == 0)
                        // {
                        //     motor_forward(&motor1, 100);
                        //     motor_forward(&motor2, 100);
                        //     ESP_LOGI(TAG, "FULL SPEED AHEAD");
                        // }
                        // else 
                        if (sensor_snapshot[2] && sensor_snapshot[3])
                        {
                            motor_forward(&motor1, 100);
                            motor_forward(&motor2, 100);
                        }
                        else
                        {
                            #ifdef ACTIVE_DEBUG
                                ESP_LOGI(TAG, "ATTACK: lost target, switching to PATROL");
                            #endif
                            last_lost_target_time = xLastWakeTime;
                            state = PATROL;
                        }
                        break;

                    default:
                        #ifdef ACTIVE_DEBUG
                            ESP_LOGW(TAG, "Mode 2: Unknown state %d", state);
                        #endif
                        break;
                }
                break;


            // MODE 3 - SAME AS MODE 1, BUT TURNING IS NOT DONE ON THE SPOT
            // AND THE ROBOT MOVES FORWARD WHILE TURNING
            case 3:
                #ifdef ACTIVE_DEBUG
                ESP_LOGI(TAG, "Mode 3 | State: %d", state);
                #endif
                switch(state)
                {
                    case PATROL:
                        if(any_detected)
                        {
                            #ifdef ACTIVE_DEBUG
                            ESP_LOGI(TAG, "PATROL: target detected, switching to CENTER");
                            #endif
                            state = CENTER;
                        }
                        else
                        {
                            TickType_t now = xTaskGetTickCount();
                            TickType_t elapsed = now - patrol_last_switch_time;

                            if(patrol_moving && elapsed >= patrol_move_duration)
                            {
                                patrol_moving = false;
                                patrol_last_switch_time = now;
                            }
                            else if(!patrol_moving && elapsed >= (patrol_cycle_duration - patrol_move_duration))
                            {
                                patrol_moving = true;
                                patrol_last_switch_time = now;
                            }

                            if(patrol_moving)
                            {
                                #ifdef ACTIVE_DEBUG
                                ESP_LOGI(TAG, "PATROL: moving forward");
                                #endif
                                motor_forward(&motor1, PATROL_SPEED);
                                motor_forward(&motor2, PATROL_SPEED);
                            }
                            else
                            {
                                #ifdef ACTIVE_DEBUG
                                ESP_LOGI(TAG, "PATROL: paused");
                                #endif
                                motor_brake(&motor1);
                                motor_brake(&motor2);
                            }
                        }
                        break;

                    case CENTER:
                        if(mid_sum > 0)
                        {
                            state = ATTACK;
                            break;
                        }
                        else if(left_sum > right_sum)
                        {
                            int boost = right_weight;
                            motor_forward(&motor1, TURN_SPEED + boost);
                            motor_reverse(&motor2, TURN_SPEED + boost);
                            curr_direction = 1;
                        }
                        else if(right_sum > left_sum)
                        {
                            int boost = left_weight;
                            motor_forward(&motor2, TURN_SPEED + boost);
                            motor_reverse(&motor1, TURN_SPEED + boost);
                            curr_direction = -1;
                        }
                        else
                            state = PATROL;
                        break;

                    case ATTACK:
                        #ifdef ACTIVE_DEBUG
                        ESP_LOGI(TAG, "ATTACK: forward(90, 90)");
                        #endif
                        if(sensor_snapshot[2] && sensor_snapshot[3])
                        {
                            motor_forward(&motor1, 100);
                            motor_forward(&motor2, 100);
                        }
                        else
                        {
                            #ifdef ACTIVE_DEBUG
                            ESP_LOGI(TAG, "ATTACK: lost target, switching to PATROL");
                            #endif
                            last_lost_target_time = xLastWakeTime;
                            state = PATROL;
                        }
                        break;

                    default:
                        #ifdef ACTIVE_DEBUG
                        ESP_LOGW(TAG, "Mode 3: Unknown state %d", state);
                        #endif
                        break;
                }
                break;


            case 4:
                #ifdef ACTIVE_DEBUG
                ESP_LOGI(TAG, "Mode 4 | State: %d", state);
                #endif
                switch(state)
                {
                    case PATROL:
                        if(any_detected)
                        {
                            #ifdef ACTIVE_DEBUG
                            ESP_LOGI(TAG, "PATROL: target detected, switching to CENTER");
                            #endif
                            state = CENTER;
                        }
                        else
                        {
                            #ifdef ACTIVE_DEBUG
                            ESP_LOGI(TAG, "PATROL: idle");
                            #endif
                            motor_brake(&motor1);
                            motor_brake(&motor2);
                        }
                        break;

                    case CENTER:
                        if(mid_sum > 0)
                        {
                            state = ATTACK;
                            break;
                        }
                        else if(left_sum > right_sum)
                        {
                            int boost = right_weight;
                            motor_forward(&motor1, TURN_SPEED + boost);
                            motor_reverse(&motor2, TURN_SPEED + boost);
                            curr_direction = 1;
                        }
                        else if(right_sum > left_sum)
                        {
                            int boost = left_weight;
                            motor_forward(&motor2, TURN_SPEED + boost);
                            motor_reverse(&motor1, TURN_SPEED + boost);
                            curr_direction = -1;
                        }
                        else
                            state = PATROL;
                        break;

                    case ATTACK:
                        #ifdef ACTIVE_DEBUG
                        ESP_LOGI(TAG, "ATTACK: forward(90, 90)");
                        #endif
                        if(sensor_snapshot[2] && sensor_snapshot[3])
                        {
                            motor_forward(&motor1, 100);
                            motor_forward(&motor2, 100);
                        }
                        else
                        {
                            #ifdef ACTIVE_DEBUG
                            ESP_LOGI(TAG, "ATTACK: lost target, switching to PATROL");
                            #endif
                            last_lost_target_time = xLastWakeTime;
                            state = PATROL;
                        }
                        break;

                    default:
                        #ifdef ACTIVE_DEBUG
                        ESP_LOGW(TAG, "Mode 4: Unknown state %d", state);
                        #endif
                        break;
                }
                break;
            
            case 5:
                #ifdef ACTIVE_DEBUG
                ESP_LOGI(TAG, "Mode 5 | State: CHARGE");
                #endif
                motor_forward(&motor1, 100);
                motor_forward(&motor2, 100);
                break;


            // ONLY CENTER ON THE OPPOINENT (ROTATING ON THE SPOT)
            case 6:
                #ifdef ACTIVE_DEBUG
                    ESP_LOGI(TAG, "Mode 4 | State: %d", state);
                #endif
                switch(state)
                {
                    case CENTER:
                        if(left_sum > right_sum && mid_sum != 2)
                        {
                            int boost = right_weight; // because we’re rotating right
                            #ifdef ACTIVE_DEBUG
                                ESP_LOGI(TAG, "CENTER: rotating right (boost %d)", boost);
                            #endif
                            motor_forward(&motor1, TURN_SPEED + boost);
                            motor_reverse(&motor2, TURN_SPEED + boost);
                            curr_direction = 1;
                        }
                        else if(right_sum > left_sum && mid_sum != 2)
                        {
                            int boost = left_weight; // rotating left
                            #ifdef ACTIVE_DEBUG
                                ESP_LOGI(TAG, "CENTER: rotating left (boost %d)", boost);
                            #endif
                            motor_forward(&motor2, TURN_SPEED + boost);
                            motor_reverse(&motor1, TURN_SPEED + boost);
                            curr_direction = -1;
                        }
                        else if(mid_sum == 2)
                        {
                            motor_brake(&motor1);
                            motor_brake(&motor2);
                        }
                        else
                        {
                            if(curr_direction == 1)
                            {
                                motor_forward(&motor1, TURN_SPEED);
                                motor_reverse(&motor2, TURN_SPEED);
                            }
                            else
                            {
                                motor_forward(&motor2, TURN_SPEED);
                                motor_reverse(&motor1, TURN_SPEED);
                            }
                        }
                        break;
                    default:
                        #ifdef ACTIVE_DEBUG
                            ESP_LOGW(TAG, "Mode 4: Unknown state %d", state);
                        #endif
                        break;

                }        
        }
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }


    #ifdef ACTIVE_DEBUG
        ESP_LOGI(TAG, "Task exiting. is_running = 0");
    #endif
}


void app_main(void) 
{
    int setup_status = setup();

    //if setup ok, blink 3 times
    if(setup_status == 1)
        blink_led(3, 150);
    else
        gpio_set_level(ONBOARD_LED, 1);

    mode_select();

    #ifdef ACTIVE_DEBUG
    ESP_LOGI(MAIN_TAG, "Mode %d selected", mode);
    #endif

    #ifdef USE_START_STOP_MODULE
    while(1)
    {
        is_running = gpio_get_level(START_STOP_MODULE);
        if(is_running)
        {
            #ifdef ACTIVE_DEBUG
            ESP_LOGI(MAIN_TAG, "Start/Stop module pressed");
            #endif
            // wait for 5 ms
            vTaskDelay(pdMS_TO_TICKS(10));
            break;
        }
        #ifdef ACTIVE_DEBUG
        ESP_LOGI("start stop", "%d", gpio_get_level(START_STOP_MODULE));
        #endif
    }
    #endif

    
    sensor_data_mutex = xSemaphoreCreateMutex();
    is_running_mutex = xSemaphoreCreateMutex();
    line_sensor_mutex = xSemaphoreCreateMutex();
    TaskHandle_t gpio_task_handle  = NULL;
    xTaskCreate(gpio_update_task, "gpio_update_task", 4096, NULL, 10, &gpio_task_handle);
    TaskHandle_t machine_state_task_handle = NULL;
    xTaskCreate(state_machine_task, "state_machine_task", 4096, NULL, 10, &machine_state_task_handle);

    int local_is_running = 0;

    while(1) {
        if (xSemaphoreTake(is_running_mutex, (TickType_t) 10) == pdTRUE)
        {
            local_is_running = is_running;
            xSemaphoreGive(is_running_mutex);
        }

        if(!local_is_running) {
            // stop all tasks
            vTaskDelete(gpio_task_handle);
            vTaskDelete(machine_state_task_handle);
            motor_coast(&motor1);
            motor_coast(&motor2);
            break;
        }
    }
    return;
}