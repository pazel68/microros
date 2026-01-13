#include <string.h>
#include <stdio.h>
#include <inttypes.h>
#include <unistd.h>
#include <math.h>
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "driver/uart.h"
#include <rmw_microros/time_sync.h>
#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/imu.h>
#include <rosidl_runtime_c/string_functions.h>

#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include "esp32_serial_transport.h"

#define TAG "TurtleBot3_Controller"

// ---------------- TurtleBot3 Motor & Encoder ----------------
// L298N Motor
#define Fw_A 23
#define Fw_B 18
#define Bw_A 19
#define Bw_B 4
#define EN_A 33
#define EN_B 32
#define ENA_PWM_CH LEDC_CHANNEL_0
#define ENB_PWM_CH LEDC_CHANNEL_1
#define PWM_FREQ 25000
#define PWM_RES LEDC_TIMER_8_BIT

// motor A (left)
#define LEFT_FWD_GPIO   Fw_A      // 23
#define LEFT_BWD_GPIO   Bw_A      // 19
#define LEFT_PWM_GPIO   EN_A      // 33
#define LEFT_PWM_CH     ENA_PWM_CH

// motor B (right)
#define RIGHT_FWD_GPIO  Fw_B      // 18
#define RIGHT_BWD_GPIO  Bw_B      // 4
#define RIGHT_PWM_GPIO  EN_B      // 32
#define RIGHT_PWM_CH    ENB_PWM_CH

#define ENC_LEFT_A      27
#define ENC_LEFT_B      13
#define ENC_RIGHT_A     25
#define ENC_RIGHT_B     26

#define WHEEL_RADIUS    0.03375     // meters (TurtleBot3 Burger wheel radius)
#define WHEEL_BASE      0.2157      // meters (TurtleBot3 Burger wheel base)
#define TICKS_PER_REV   980      // Ticks per revolution for TurtleBot3 motors
#define MAX_WHEEL_VELOCITY 0.25   // m/s

volatile int32_t left_ticks = 0;
volatile int32_t right_ticks = 0;

#define LEFT_DIR    1       // forward = +1
#define RIGHT_DIR   1       

#define MAX_PWM     255

// ---------------- Razor IMU UART Config ----------------
// ใช้ UART2 เพื่อไม่ให้ชนกับ micro-ROS (UART0)
#define IMU_TXD_PIN (GPIO_NUM_21) // ต่อเข้า RX ของ Razor
#define IMU_RXD_PIN (GPIO_NUM_22) // ต่อเข้า TX ของ Razor
#define IMU_UART_NUM UART_NUM_2
#define IMU_BAUD_RATE 57600
#define IMU_BUF_SIZE 1024

// Constants for Unit Conversion
#define DEG_TO_RAD (0.0174532925)
#define GRAVITY_EARTH (9.80665)
#define ACCEL_SCALE (256.0) // Razor 1 unit = 1/256 G

rcl_publisher_t encoder_pub;
std_msgs__msg__Int32MultiArray encoder_msg;

// ---------------- Micro-ROS IMU ----------------
rcl_publisher_t imu_pub;
sensor_msgs__msg__Imu imu_msg;

//global variable
volatile float target_linear_v = 0.0f;
volatile float target_angular_v = 0.0f;

// static globals (declare นอก callback)
static float integral_left = 0.0f, integral_right = 0.0f;
static float prev_error_left = 0.0f, prev_error_right = 0.0f;

// ---------------- Macro ตรวจ error ----------------
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

// ---------------- UART Init for Razor IMU ----------------
void init_imu_uart() {
    const uart_config_t uart_config = {
        .baud_rate = IMU_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    // Install UART driver (UART2)
    ESP_ERROR_CHECK(uart_driver_install(IMU_UART_NUM, IMU_BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(IMU_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(IMU_UART_NUM, IMU_TXD_PIN, IMU_RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

void start_razor_imu() {
    // Send commands to configure Razor IMU
    uart_write_bytes(IMU_UART_NUM, "#o0", 3); // Stop streaming
    vTaskDelay(pdMS_TO_TICKS(50));
    uart_write_bytes(IMU_UART_NUM, "#ox", 3); // Set Output to #YPRAG (Angles + Accel + Gyro)
    vTaskDelay(pdMS_TO_TICKS(50));
    uart_write_bytes(IMU_UART_NUM, "#o1", 3); // Start streaming
}

// ---------------- Helper: Convert Euler to Quaternion ----------------
void euler_to_quat(float roll_deg, float pitch_deg, float yaw_deg, geometry_msgs__msg__Quaternion *q) {
    float roll = roll_deg * DEG_TO_RAD;
    float pitch = pitch_deg * DEG_TO_RAD;
    float yaw = yaw_deg * DEG_TO_RAD;

    float cy = cosf(yaw * 0.5f);
    float sy = sinf(yaw * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);
    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);

    q->w = cr * cp * cy + sr * sp * sy;
    q->x = sr * cp * cy - cr * sp * sy;
    q->y = cr * sp * cy + sr * cp * sy;
    q->z = cr * cp * sy - sr * sp * cy;
}

// ---------------- Interrupt Service Routines (ISRs) for Encoders ----------------
void IRAM_ATTR encoder_isr_left() {
    int a = gpio_get_level(ENC_LEFT_A);
    int b = gpio_get_level(ENC_LEFT_B);
    if(a == b) { left_ticks += LEFT_DIR; } else { left_ticks -= LEFT_DIR; }
}

void IRAM_ATTR encoder_isr_right() {
    int a = gpio_get_level(ENC_RIGHT_A);
    int b = gpio_get_level(ENC_RIGHT_B);
    if(a == b) { right_ticks -= RIGHT_DIR; } else { right_ticks += RIGHT_DIR; }
}

// ---------------- Initialize Encoders ----------------
static void encoder_init(void)
{
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_ANYEDGE,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pin_bit_mask = (1ULL << ENC_LEFT_A) | (1ULL << ENC_LEFT_B) |
                        (1ULL << ENC_RIGHT_A) | (1ULL << ENC_RIGHT_B),
    };
    gpio_config(&io_conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(ENC_LEFT_A, encoder_isr_left, NULL);
    gpio_isr_handler_add(ENC_RIGHT_A, encoder_isr_right, NULL);
}

// ---------------- Initialize PWM for motors ----------------
static void ledc_init(void)
{
    ledc_timer_config_t pwm_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .duty_resolution  = PWM_RES,
        .timer_num        = LEDC_TIMER_0,
        .freq_hz          = PWM_FREQ,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&pwm_timer));

    ledc_channel_config_t left_channel = {
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = LEFT_PWM_CH,
        .timer_sel      = LEDC_TIMER_0,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEFT_PWM_GPIO,
        .duty           = 0,
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&left_channel));

    ledc_channel_config_t right_channel = {
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = RIGHT_PWM_CH,
        .timer_sel      = LEDC_TIMER_0,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = RIGHT_PWM_GPIO,
        .duty           = 0,
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&right_channel));
}

// ---------------- Motor Control Function ----------------
int limit_pwm(int value){
    if(value > MAX_PWM) return MAX_PWM;
    if(value < -MAX_PWM) return -MAX_PWM;
    return value;
}

void motor_set(int left_pwm, int right_pwm) {
    left_pwm = limit_pwm(left_pwm);
    right_pwm = limit_pwm(right_pwm);

    if (left_pwm >= 0) {
        gpio_set_level(LEFT_FWD_GPIO, 1);
        gpio_set_level(LEFT_BWD_GPIO, 0);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEFT_PWM_CH, left_pwm);
    } else {
        gpio_set_level(LEFT_FWD_GPIO, 0);
        gpio_set_level(LEFT_BWD_GPIO, 1);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEFT_PWM_CH, -left_pwm);
    }
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEFT_PWM_CH);

    if (right_pwm >= 0) {
        gpio_set_level(RIGHT_FWD_GPIO, 1);
        gpio_set_level(RIGHT_BWD_GPIO, 0);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, RIGHT_PWM_CH, right_pwm);
    } else {
        gpio_set_level(RIGHT_FWD_GPIO, 0);
        gpio_set_level(RIGHT_BWD_GPIO, 1);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, RIGHT_PWM_CH, -right_pwm);
    }
    ledc_update_duty(LEDC_LOW_SPEED_MODE, RIGHT_PWM_CH);
}

void cmd_vel_callback(const void * msgin)
{
    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
    target_linear_v = msg->linear.x;
    target_angular_v = msg->angular.z;
}

// ---------------- Timer Callback (PID + IMU Read) ----------------
void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer == NULL) return;
    
    // 1. [สำคัญ] จับเวลา "ครั้งเดียว" ที่ต้นฟังก์ชัน เพื่อใช้กับทุก Sensor
    int64_t time_now_ns = rmw_uros_epoch_nanos();

    float dt = 0.02f; // 20 ms timer

    // === PART 1: Motor Control (PID) ===
    static int32_t prev_left = 0, prev_right = 0;
    int32_t delta_left  = left_ticks - prev_left;
    int32_t delta_right = right_ticks - prev_right;
    prev_left  = left_ticks;
    prev_right = right_ticks;

    float distance_per_tick = (2.0f * M_PI * WHEEL_RADIUS) / TICKS_PER_REV;
    float v_left  = delta_left  * distance_per_tick / dt;
    float v_right = delta_right * distance_per_tick / dt;

    float v_left_target  = target_linear_v - (target_angular_v * WHEEL_BASE / 2.0f);
    float v_right_target = target_linear_v + (target_angular_v * WHEEL_BASE / 2.0f);

    // PID parameters (ปรับจูนตามจริง)
    float Kp = 0.0f;
    float Ki = 0.0f;
    float Kd = 0.0f;
    
    const float INTEGRAL_LIMIT = 5000.0f;   
    const float ERROR_INTEGRATE_TH = 0.5f;  
    const int MIN_PWM = 180;                 

    // Left PID
    float error_left = v_left_target - v_left;
    if (fabsf(error_left) < ERROR_INTEGRATE_TH && fabsf(v_left_target) > 0.01f) {
        integral_left += error_left * dt;
    } else {
        integral_left *= 0.999f;
    }
    if (integral_left > INTEGRAL_LIMIT) integral_left = INTEGRAL_LIMIT;
    if (integral_left < -INTEGRAL_LIMIT) integral_left = -INTEGRAL_LIMIT;
    float derivative_left = (error_left - prev_error_left) / dt;
    prev_error_left = error_left;
    float ff_left = (v_left_target / MAX_WHEEL_VELOCITY) * MAX_PWM;
    int pwm_left = (int)(ff_left + Kp * error_left + Ki * integral_left + Kd * derivative_left);

    // Right PID
    float error_right = v_right_target - v_right;
    if (fabsf(error_right) < ERROR_INTEGRATE_TH && fabsf(v_right_target) > 0.01f) {
        integral_right += error_right * dt;
    } else {
        integral_right *= 0.999f;
    }
    if (integral_right > INTEGRAL_LIMIT) integral_right = INTEGRAL_LIMIT;
    if (integral_right < -INTEGRAL_LIMIT) integral_right = -INTEGRAL_LIMIT;
    float derivative_right = (error_right - prev_error_right) / dt;
    prev_error_right = error_right;
    float ff_right = (v_right_target / MAX_WHEEL_VELOCITY) * MAX_PWM;
    int pwm_right = (int)(ff_right + Kp * error_right + Ki * integral_right + Kd * derivative_right);

    // Deadzone & Calibration
    if (pwm_left > 0 && pwm_left < MIN_PWM) pwm_left = MIN_PWM;
    if (pwm_left < 0 && pwm_left > -MIN_PWM) pwm_left = -MIN_PWM;
    if (pwm_right > 0 && pwm_right < MIN_PWM) pwm_right = MIN_PWM;
    if (pwm_right < 0 && pwm_right > -MIN_PWM) pwm_right = -MIN_PWM;
    
    const float LEFT_GAIN  = 1.00f;   
    const float RIGHT_GAIN = 1.00f;   
    pwm_left  = (int)(pwm_left  * LEFT_GAIN);
    pwm_right = (int)(pwm_right * RIGHT_GAIN);
    
    motor_set(pwm_left, pwm_right);

    if (fabsf(target_linear_v) < 0.01f) {
        integral_left = 0.0f;
        integral_right = 0.0f;
    }

    // Publish Encoder
    // ขยาย Array เป็น 4 ช่อง: [Left, Right, Time_Sec, Time_Nanosec]
    encoder_msg.data.data[0] = left_ticks;
    encoder_msg.data.data[1] = right_ticks;
    
    // แยกเวลาเป็นวินาทีและนาโนวินาที (เพราะ Int32 เก็บ int64 ไม่พอ)
    encoder_msg.data.data[2] = (int32_t)(time_now_ns / 1000000000); // Seconds
    encoder_msg.data.data[3] = (int32_t)(time_now_ns % 1000000000); // Nanoseconds
    
    RCSOFTCHECK(rcl_publish(&encoder_pub, &encoder_msg, NULL));

    // === PART 2: Razor IMU Reading (UART) ===
    uint8_t data[256];
    // อ่าน UART แบบไม่บล็อกนาน (timeout 5ms) เพื่อไม่ให้กระทบ Motor Loop 20ms
    int length = uart_read_bytes(IMU_UART_NUM, data, 255, pdMS_TO_TICKS(5));

    if (length > 0) {
        data[length] = '\0';
        // คาดหวัง Format: #YPRAG=yaw,pitch,roll,ax,ay,az,gx,gy,gz
        char *start = strstr((char*)data, "#YPRAG=");
        if (start != NULL) {
            float y, p, r;       // Euler (Deg)
            float ax, ay, az;    // Accel Raw (1/256 G)
            float gx, gy, gz;    // Gyro (rad/s)

            // แกะค่า 9 ตัว
            if (sscanf(start + 7, "%f,%f,%f,%f,%f,%f,%f,%f,%f", 
                       &y, &p, &r, 
                       &ax, &ay, &az, 
                       &gx, &gy, &gz) == 9) {
                
                // init msg
                sensor_msgs__msg__Imu__init(&imu_msg);

                // 1. Orientation (Euler -> Quaternion)
                euler_to_quat(r, -p, -y, &imu_msg.orientation);

                // 2. Linear Acceleration (Convert 1/256G -> m/s^2)
                imu_msg.linear_acceleration.x = -ax * (GRAVITY_EARTH / ACCEL_SCALE);
                imu_msg.linear_acceleration.y = ay * (GRAVITY_EARTH / ACCEL_SCALE);
                imu_msg.linear_acceleration.z = az * (GRAVITY_EARTH / ACCEL_SCALE);

                // 3. Angular Velocity (rad/s -> rad/s)
                imu_msg.angular_velocity.x = gx;
                imu_msg.angular_velocity.y = -gy;
                imu_msg.angular_velocity.z = -gz;

                // Covariance (บอก ROS ว่าเชื่อค่าไหน)
                double imu_cov = 0.2;
                imu_msg.orientation_covariance[0] = imu_cov; 
                imu_msg.orientation_covariance[4] = imu_cov;
                imu_msg.orientation_covariance[8] = imu_cov;
                
                imu_msg.angular_velocity_covariance[0] = imu_cov; 
                imu_msg.angular_velocity_covariance[4] = imu_cov; 
                imu_msg.angular_velocity_covariance[8] = imu_cov;
                
                imu_msg.linear_acceleration_covariance[0] = imu_cov;
                imu_msg.linear_acceleration_covariance[4] = imu_cov;
                imu_msg.linear_acceleration_covariance[8] = imu_cov;

                // Timestamp & Frame ID
                int64_t t_ns = rmw_uros_epoch_nanos();
                //ใช้เวลา time_now_ns ตัวเดียวกับ Encoder!
                imu_msg.header.stamp.sec = (int32_t)(time_now_ns / 1000000000);
                imu_msg.header.stamp.nanosec = (uint32_t)(time_now_ns % 1000000000);
                rosidl_runtime_c__String__assign(&imu_msg.header.frame_id, "imu_link");

                RCSOFTCHECK(rcl_publish(&imu_pub, &imu_msg, NULL));
            }
        }
    }
}

// ---------------- micro-ROS Task ----------------
void micro_ros_task(void * arg)
{
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    
    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "turtlebot3_esp32_controller", "", &support));
    
    // เริ่มต้น UART สำหรับ Razor และสั่งเริ่มทำงาน
    init_imu_uart();
    start_razor_imu();

    RCCHECK(rclc_publisher_init_default(&imu_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu"));
   
    // Publisher for Encoder Ticks
    RCCHECK(rclc_publisher_init_default(
        &encoder_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
        "encoder_ticks"
    ));

    // Subscription to /cmd_vel
    rcl_subscription_t cmd_vel_sub;
    geometry_msgs__msg__Twist cmd_vel_msg;
    geometry_msgs__msg__Twist__init(&cmd_vel_msg);
    RCCHECK(rclc_subscription_init_default(
        &cmd_vel_sub, 
        &node, 
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), 
        "/cmd_vel"
    ));

    // Timer (20ms -> 50Hz)
    rcl_timer_t timer;
    const unsigned int timer_timeout = 20;
    RCCHECK(rclc_timer_init_default2(&timer, &support, RCL_MS_TO_NS(timer_timeout), timer_callback, true));

    // Encoder message initialization
    std_msgs__msg__Int32MultiArray__init(&encoder_msg);
    encoder_msg.data.size = 4;
    encoder_msg.data.capacity = 4;
    encoder_msg.data.data = malloc(sizeof(int32_t) * 4);

    // Executor
    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));
    RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA));    

    while(1){
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
        
        // สั่ง Sync เวลาทุกรอบ หรือทุกๆ ระยะเวลาหนึ่ง
        // ถ้า Agent เชื่อมต่ออยู่ เวลาของ ESP32 จะถูกปรับให้ตรงกับ PC
        rmw_uros_sync_session(10);
        
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // Cleanup
    RCCHECK(rcl_publisher_fini(&encoder_pub, &node));
    RCCHECK(rcl_node_fini(&node));
    vTaskDelete(NULL);
}
    
// ---------------- Main Function ----------------
static size_t uart_port = UART_NUM_0;
void app_main(void)
{
#if defined(RMW_UXRCE_TRANSPORT_CUSTOM)
    rmw_uros_set_custom_transport(
        true,
        (void *) &uart_port,
        esp32_serial_open,
        esp32_serial_close,
        esp32_serial_write,
        esp32_serial_read
    );
#else
#error micro-ROS transports misconfigured
#endif  // RMW_UXRCE_TRANSPORT_CUSTOM

    // Init motor GPIO pins
    gpio_reset_pin(LEFT_FWD_GPIO);
    gpio_set_direction(LEFT_FWD_GPIO, GPIO_MODE_OUTPUT);
    gpio_reset_pin(LEFT_BWD_GPIO);
    gpio_set_direction(LEFT_BWD_GPIO, GPIO_MODE_OUTPUT);
    gpio_reset_pin(RIGHT_FWD_GPIO);
    gpio_set_direction(RIGHT_FWD_GPIO, GPIO_MODE_OUTPUT);
    gpio_reset_pin(RIGHT_BWD_GPIO);
    gpio_set_direction(RIGHT_BWD_GPIO, GPIO_MODE_OUTPUT);

    // Init PWM
    ledc_init();

    // Init Encoders
    encoder_init();     

    // Start micro-ROS task
    xTaskCreate(micro_ros_task, "uros_task", CONFIG_MICRO_ROS_APP_STACK, NULL, CONFIG_MICRO_ROS_APP_TASK_PRIO, NULL);
}
