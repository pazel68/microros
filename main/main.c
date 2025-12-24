#include <string.h>
#include <stdio.h>
#include <inttypes.h>
#include <unistd.h>
#include <math.h>
#include "driver/i2c.h"
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
#define WHEEL_BASE      0.2157     // meters (TurtleBot3 Burger wheel base)
#define TICKS_PER_REV   980      // Ticks per revolution for TurtleBot3 motors
#define MAX_WHEEL_VELOCITY 0.25   // m/s (ความเร็วสูงสุดของล้อ, ปรับค่านี้ได้ตามสเปคหุ่นยนต์)

volatile int32_t left_ticks = 0;
volatile int32_t right_ticks = 0;

#define LEFT_DIR    1      // forward = +1
#define RIGHT_DIR   1      // NOTE: Your original code had different logic for each ISR. This should be consistent.

#define MAX_PWM     255

// ---------------- I2C config ----------------
#define I2C_MASTER_SCL_IO  22
#define I2C_MASTER_SDA_IO  21
#define I2C_MASTER_NUM     I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 400000

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

// ---------------- I2C helper ----------------
static esp_err_t i2c_wr(uint8_t addr, uint8_t reg, uint8_t data) {
    uint8_t buf[2] = {reg, data};
    return i2c_master_write_to_device(I2C_MASTER_NUM, addr, buf, 2, 20/portTICK_PERIOD_MS);
}

static esp_err_t i2c_rd(uint8_t addr, uint8_t reg, uint8_t *data, size_t len) {
    return i2c_master_write_read_device(I2C_MASTER_NUM, addr, &reg, 1, data, len, 20/portTICK_PERIOD_MS);
}

// ---------------- MPU-9250 registers ----------------
#define MPU_ADDR          0x68
#define REG_PWR_MGMT_1    0x6B
#define REG_INT_PIN_CFG   0x37   // BYPASS_EN bit
#define REG_USER_CTRL     0x6A   // I2C_MST_EN bit
#define REG_ACCEL_XOUT_H  0x3B
#define REG_GYRO_XOUT_H   0x43
#define REG_WHO_AM_I      0x75

// ---------------- AK8963 (Magnetometer) ----------------
#define MAG_ADDR     0x0C
#define MAG_WIA      0x00
#define MAG_ST1      0x02
#define MAG_HXL      0x03
#define MAG_ST2      0x09
#define MAG_CNTL1    0x0A
#define MAG_CNTL2    0x0B
#define MAG_ASAX     0x10

#define AK8963_16BIT       0x10
#define AK8963_CONT_8HZ    0x02
#define AK8963_CONT_100HZ  0x06
#define AK8963_FUSE_ROM    0x0F
#define AK8963_POWER_DOWN  0x00

// ---------------- scale factors ----------------
#define ACC_LSB_2G   16384.0f
#define GYRO_LSB_250 131.0f
#define DEG2RAD      0.017453292519943295f
#define G_SI         9.80665f
#define MAG_LSB_16BIT_UT 0.15f

// ---------------- global magnetometer adjustment ----------------
static float asa_adj[3] = {1,1,1};

// ---------------- Mahony Filter ----------------
typedef struct {
    float twoKp, twoKi; 
    float q0, q1, q2, q3;
    float ix, iy, iz;
} MahonyAHRS;

static MahonyAHRS mahony = { 2.0f*0.5f, 2.0f*0.1f, 1,0,0,0, 0,0,0 };

// ---------------- Macro ตรวจ error ----------------
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

//funtion added
// ---------------- Mahony update function ----------------
static void MahonyUpdate(float gx, float gy, float gz,
                         float ax, float ay, float az,
                         float mx, float my, float mz, float dt)
{
    // normalize accelerometer
    float norm = sqrtf(ax*ax + ay*ay + az*az);
    if (norm > 0.0f) { ax/=norm; ay/=norm; az/=norm; }

    // normalize magnetometer
    norm = sqrtf(mx*mx + my*my + mz*mz);
    if (norm > 0.0f) { mx/=norm; my/=norm; mz/=norm; }

    // ตัวคำนวณ quaternion orientation (Mahony AHRS)
    float q0=mahony.q0,q1=mahony.q1,q2=mahony.q2,q3=mahony.q3;
    float q0q0=q0*q0, q0q1=q0*q1, q0q2=q0*q2, q0q3=q0*q3;
    float q1q1=q1*q1, q1q2=q1*q2, q1q3=q1*q3;
    float q2q2=q2*q2, q2q3=q2*q3, q3q3=q3*q3;

    // reference direction of Earth's magnetic field
    float hx = 2.0f*(mx*(0.5f - q2q2 - q3q3) + my*(q1q2 - q0q3) + mz*(q1q3 + q0q2));
    float hy = 2.0f*(mx*(q1q2 + q0q3) + my*(0.5f - q1q1 - q3q3) + mz*(q2q3 - q0q1));
    float bx = sqrtf(hx*hx + hy*hy);
    float bz = 2.0f*(mx*(q1q3 - q0q2) + my*(q2q3 + q0q1) + mz*(0.5f - q1q1 - q2q2));

    // estimated direction of gravity and magnetic field
    float halfvx = q1q3 - q0q2;
    float halfvy = q0q1 + q2q3;
    float halfvz = q0q0 - 0.5f + q3q3;
    float halfwx = bx*(0.5f - q2q2 - q3q3) + bz*(q1q3 - q0q2);
    float halfwy = bx*(q1q2 - q0q3) + bz*(q0q1 + q2q3);
    float halfwz = bx*(q0q2 + q1q3) + bz*(0.5f - q1q1 - q2q2);

    // error between estimated and measured direction
    float halfex = (ay*halfvz - az*halfvy) + (my*halfwz - mz*halfwy);
    float halfey = (az*halfvx - ax*halfvz) + (mz*halfwx - mx*halfwz);
    float halfez = (ax*halfvy - ay*halfvx) + (mx*halfwy - my*halfwx);

    // integral feedback
    mahony.ix += mahony.twoKi * halfex * dt;
    mahony.iy += mahony.twoKi * halfey * dt;
    mahony.iz += mahony.twoKi * halfez * dt;

    // proportional + integral
    gx += mahony.twoKp * halfex + mahony.ix;
    gy += mahony.twoKp * halfey + mahony.iy;
    gz += mahony.twoKp * halfez + mahony.iz;

    // integrate quaternion rate
    gx *= 0.5f*dt; gy*=0.5f*dt; gz*=0.5f*dt;
    float qa=q0, qb=q1, qc=q2;
    q0 += (-qb*gx - qc*gy - q3*gz);
    q1 += (qa*gx + qc*gz - q3*gy);
    q2 += (qa*gy - qb*gz + q3*gx);
    q3 += (qa*gz + qb*gy - qc*gx);

    // normalize quaternion
    norm = 1.0f/sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    mahony.q0 = q0*norm; mahony.q1 = q1*norm; mahony.q2 = q2*norm; mahony.q3 = q3*norm;
}

// ---------------- Convert quaternion -> roll/pitch/yaw ----------------
static void quat_to_rpy(float *roll, float *pitch, float *yaw) {
    float q0=mahony.q0,q1=mahony.q1,q2=mahony.q2,q3=mahony.q3;
    *roll  = atan2f(2.f*(q0*q1 + q2*q3), 1.f - 2.f*(q1*q1 + q2*q2));
    *pitch = asinf(2.f*(q0*q2 - q3*q1));
    *yaw   = atan2f(2.f*(q0*q3 + q1*q2), 1.f - 2.f*(q2*q2 + q3*q3));
}

// ---------------- I2C init MPU ----------------
static void mpu_init(void) {
    i2c_wr(MPU_ADDR, REG_PWR_MGMT_1, 0x00); // wake up
    vTaskDelay(pdMS_TO_TICKS(10));
    i2c_wr(MPU_ADDR, REG_USER_CTRL, 0x00);  // disable I2C master
    i2c_wr(MPU_ADDR, REG_INT_PIN_CFG, 0x02); // BYPASS_EN = 1
}

// ---------------- Read AK8963 fuse ROM ----------------
static void ak8963_read_asa(float adj[3]) {
    i2c_wr(MAG_ADDR, MAG_CNTL1, AK8963_POWER_DOWN); vTaskDelay(pdMS_TO_TICKS(10));
    i2c_wr(MAG_ADDR, MAG_CNTL1, AK8963_FUSE_ROM); vTaskDelay(pdMS_TO_TICKS(10));

    uint8_t asa[3] = {0};
    i2c_rd(MAG_ADDR, MAG_ASAX, asa, 3);
    for (int i=0;i<3;i++) adj[i] = ((float)asa[i]-128)/256.0f + 1.0f;

    i2c_wr(MAG_ADDR, MAG_CNTL1, AK8963_POWER_DOWN); vTaskDelay(pdMS_TO_TICKS(10));
}

// ---------------- Start AK8963 continuous 100Hz ----------------
static void ak8963_start_16bit_100hz(void) {
    i2c_wr(MAG_ADDR, MAG_CNTL2, 0x01); vTaskDelay(pdMS_TO_TICKS(10)); // soft reset
    i2c_wr(MAG_ADDR, MAG_CNTL1, AK8963_POWER_DOWN); vTaskDelay(pdMS_TO_TICKS(10));
    i2c_wr(MAG_ADDR, MAG_CNTL1, AK8963_16BIT | AK8963_CONT_100HZ); vTaskDelay(pdMS_TO_TICKS(10));
}

// ---------------- Read raw accel/gyro ----------------
static bool mpu_read_raw(int16_t *accel, int16_t *gyro) {
    uint8_t buf[14];
    if (i2c_rd(MPU_ADDR, REG_ACCEL_XOUT_H, buf, 14)!=ESP_OK) return false;
    accel[0]=(buf[0]<<8)|buf[1];
    accel[1]=(buf[2]<<8)|buf[3];
    accel[2]=(buf[4]<<8)|buf[5];
    gyro[0]=(buf[8]<<8)|buf[9];
    gyro[1]=(buf[10]<<8)|buf[11];
    gyro[2]=(buf[12]<<8)|buf[13];
    return true;
}

// ---------------- Read AK8963 magnetometer ----------------
static bool ak8963_read_mxyz(int16_t *mx_my_mz, uint8_t *st2) {
    uint8_t st1=0;
    if (i2c_rd(MAG_ADDR, MAG_ST1, &st1, 1)!=ESP_OK) return false;
    if (!(st1 & 0x01)) return false; // DRDY

    uint8_t raw[6];
    if (i2c_rd(MAG_ADDR, MAG_HXL, raw, 6)!=ESP_OK) return false;

    mx_my_mz[0] = (int16_t)((raw[1]<<8)|raw[0]);
    mx_my_mz[1] = (int16_t)((raw[3]<<8)|raw[2]);
    mx_my_mz[2] = (int16_t)((raw[5]<<8)|raw[4]);

    uint8_t st2_local=0;
    if (i2c_rd(MAG_ADDR, MAG_ST2, &st2_local, 1)!=ESP_OK) return false;
    if (st2) *st2 = st2_local;
    if (st2_local & 0x08) return false; // overflow
    return true;
}

//ใน ESP-IDF เราต้อง เรียก i2c_driver_install() หลังจากตั้งค่า i2c_config_t และ i2c_param_config()
static void i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0));
}

// ---------------- Interrupt Service Routines (ISRs) for Encoders ----------------
void IRAM_ATTR encoder_isr_left() {
    int a = gpio_get_level(ENC_LEFT_A);
    int b = gpio_get_level(ENC_LEFT_B);
    if(a == b) { // Standard quadrature encoder logic
        left_ticks += LEFT_DIR;
    } else {
        left_ticks -= LEFT_DIR;
    }
}

void IRAM_ATTR encoder_isr_right() {
    int a = gpio_get_level(ENC_RIGHT_A);
    int b = gpio_get_level(ENC_RIGHT_B);
    if(a == b) { // This logic depends on motor wiring. Adjust if wheels spin backwards.
        right_ticks -= RIGHT_DIR;
    } else {
        right_ticks += RIGHT_DIR;
    }
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


// ---------------- Callback for /cmd_vel topic ----------------
// แก้ไขใหม่: คำนวณและสั่งมอเตอร์โดยตรงจาก callback นี้
void cmd_vel_callback(const void * msgin)
{
    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;

    target_linear_v = msg->linear.x;
    target_angular_v = msg->angular.z;

}

// timer callback
void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer == NULL) return;

    float dt = 0.02f; // 20 ms timer

    // --- 1. คำนวณความเร็วปัจจุบันจาก encoder ---
    static int32_t prev_left = 0, prev_right = 0;
    int32_t delta_left  = left_ticks - prev_left;
    int32_t delta_right = right_ticks - prev_right;
    prev_left  = left_ticks;
    prev_right = right_ticks;

    float distance_per_tick = (2.0f * M_PI * WHEEL_RADIUS) / TICKS_PER_REV;
    float v_left  = delta_left  * distance_per_tick / dt;
    float v_right = delta_right * distance_per_tick / dt;

    // --- 2. คำนวณความเร็วล้อเป้าหมายจาก cmd_vel ---
    float v_left_target  = target_linear_v - (target_angular_v * WHEEL_BASE / 2.0f);
    float v_right_target = target_linear_v + (target_angular_v * WHEEL_BASE / 2.0f);

    // --- 3. ปรับ PWM แบบ PID ---
    // PID parameters
    float Kp = 0.0f;
    float Ki = 0.0f;
    float Kd = 0.0f;
    
    const float INTEGRAL_LIMIT = 5000.0f;   // ปรับตามหน่วยของ integral
    const float ERROR_INTEGRATE_TH = 0.5f;  // integrate เฉพาะเมื่อ |error| < TH (m/s) หรือปรับตามสถานการณ์
    const int MIN_PWM = 10;                 // deadzone compensation
   
   // ภายใน callback หลังคำนวณ v_left, v_left_target_adj เป็นต้น
   float error_left = v_left_target - v_left;
   float error_right = v_right_target - v_right;

   // conditional integration: integrate เฉพาะเมื่อ error ไม่ใหญ่เกินไป (หลีกเลี่ยง windup)
   if (fabsf(error_left) < ERROR_INTEGRATE_TH && fabsf(v_left_target) > 0.01f) {
       integral_left += error_left * dt;
   } else {
       // optional: decay integral เล็กน้อย เพื่อไม่ให้ค้าง
       integral_left *= 0.999f;
   }

   // clamp integral
   if (integral_left > INTEGRAL_LIMIT) integral_left = INTEGRAL_LIMIT;
   if (integral_left < -INTEGRAL_LIMIT) integral_left = -INTEGRAL_LIMIT;

   float derivative_left = (error_left - prev_error_left) / dt;
   prev_error_left = error_left;

   float ff_left = (v_left_target / MAX_WHEEL_VELOCITY) * MAX_PWM;
   int pwm_left = (int)(ff_left + Kp * error_left + Ki * integral_left + Kd * derivative_left);

   // --- same for right ---
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

   // deadzone
   if (pwm_left > 0 && pwm_left < MIN_PWM) pwm_left = MIN_PWM;
   if (pwm_left < 0 && pwm_left > -MIN_PWM) pwm_left = -MIN_PWM;
   if (pwm_right > 0 && pwm_right < MIN_PWM) pwm_right = MIN_PWM;
   if (pwm_right < 0 && pwm_right > -MIN_PWM) pwm_right = -MIN_PWM;
   
   // --- Calibration factor สำหรับชดเชยความต่างของมอเตอร์ ---
   // ถ้าล้อซ้ายแรงไป ให้ลดค่า LEFT_GAIN < 1.0
   // ถ้าล้อซ้ายอ่อนเกินไป ให้เพิ่มค่า LEFT_GAIN > 1.0
   const float LEFT_GAIN  = 1.00f;   // ตัวอย่าง: ลดแรงล้อซ้าย 5%
   const float RIGHT_GAIN = 1.00f;   // ไม่เปลี่ยนแปลง
   
   // apply calibration gain
   pwm_left  = (int)(pwm_left  * LEFT_GAIN);
   pwm_right = (int)(pwm_right * RIGHT_GAIN);
   
   // limits แล้วสั่งมอเตอร์
   pwm_left = limit_pwm(pwm_left);
   pwm_right = limit_pwm(pwm_right);
   motor_set(pwm_left, pwm_right);
   //ESP_LOGI(TAG, "v_left: %.3f, v_left_target: %.3f, pwm_left: %d\n", v_left, v_left_target, pwm_left);

   // reset integral เมื่อ stop หรือ direction change (optional)
   if (fabsf(target_linear_v) < 0.01f) {
       integral_left = 0.0f;
       integral_right = 0.0f;
   }


    // --- 4. Publish encoder ---
    encoder_msg.data.data[0] = left_ticks;
    encoder_msg.data.data[1] = right_ticks;
    RCSOFTCHECK(rcl_publish(&encoder_pub, &encoder_msg, NULL));

    // ------------------ 5. Publish IMU ทุก 2 รอบ (20 ms) ------------------
    //static int imu_counter = 0;
    //imu_counter++;
    //if (imu_counter >= 2) { // update ทุก 20ms
        //imu_counter = 0;

        // init message
        sensor_msgs__msg__Imu__fini(&imu_msg);
        sensor_msgs__msg__Imu__init(&imu_msg);

        int16_t ar[3]={0}, gr[3]={0}, mr[3]={0};
        float ax=0, ay=0, az=0;
        float gx=0, gy=0, gz=0;
        float mx=0, my=0, mz=0;

        // อ่าน MPU
        if (mpu_read_raw(ar, gr)) {
            ax = ar[0]/ACC_LSB_2G*G_SI;
            ay = ar[1]/ACC_LSB_2G*G_SI;
            az = ar[2]/ACC_LSB_2G*G_SI;
            gx = gr[0]/GYRO_LSB_250*DEG2RAD;
            gy = gr[1]/GYRO_LSB_250*DEG2RAD;
            gz = gr[2]/GYRO_LSB_250*DEG2RAD;
        }

        // อ่าน magnetometer
        uint8_t st2=0;
        if (ak8963_read_mxyz(mr,&st2)) {
            mx = mr[0]*asa_adj[0]*MAG_LSB_16BIT_UT;
            my = mr[1]*asa_adj[1]*MAG_LSB_16BIT_UT;
            mz = mr[2]*asa_adj[2]*MAG_LSB_16BIT_UT;
        }

        // update orientation quaternion
        MahonyUpdate(gx, gy, gz, ax, ay, az, mx, my, mz, dt);

        // normalize quaternion
        float qnorm = sqrtf(mahony.q0*mahony.q0 + mahony.q1*mahony.q1 + mahony.q2*mahony.q2 + mahony.q3*mahony.q3);
        imu_msg.orientation.w = mahony.q0 / qnorm;
        imu_msg.orientation.x = mahony.q1 / qnorm;
        imu_msg.orientation.y = mahony.q2 / qnorm;
        imu_msg.orientation.z = mahony.q3 / qnorm;

        imu_msg.angular_velocity.x = gx;
        imu_msg.angular_velocity.y = gy;
        imu_msg.angular_velocity.z = gz;

        imu_msg.linear_acceleration.x = ax;
        imu_msg.linear_acceleration.y = ay;
        imu_msg.linear_acceleration.z = az;
        
        // ตั้งค่าความคลาดเคลื่อน (Covariance)
        // ยิ่งเลขน้อย = ยิ่งเชื่อถือมาก
        
        // Orientation (Quaternion): มาจาก Mahony เชื่อถือได้ปานกลาง
        imu_msg.orientation_covariance[0] = 0.01; // Roll
        imu_msg.orientation_covariance[4] = 0.01; // Pitch
        imu_msg.orientation_covariance[8] = 0.01; // Yaw

        // Angular Velocity (Gyro): แม่นยำสูง เชื่อถือได้มาก
        imu_msg.angular_velocity_covariance[0] = 0.001; 
        imu_msg.angular_velocity_covariance[4] = 0.001;
        imu_msg.angular_velocity_covariance[8] = 0.001;

        // Linear Acceleration: สั่นไหวเยอะ เชื่อถือน้อยหน่อย
        imu_msg.linear_acceleration_covariance[0] = 0.1;
        imu_msg.linear_acceleration_covariance[4] = 0.1;
        imu_msg.linear_acceleration_covariance[8] = 0.1;

        // timestamp
        int64_t t_ns = rmw_uros_epoch_nanos();
        imu_msg.header.stamp.sec = t_ns / 1000000000ULL;
        imu_msg.header.stamp.nanosec = t_ns % 1000000000ULL;
        rosidl_runtime_c__String__assign(&imu_msg.header.frame_id, "imu_link");

        RCSOFTCHECK(rcl_publish(&imu_pub, &imu_msg, NULL));
    //}
}


// ---------------- micro-ROS Task ----------------
void micro_ros_task(void * arg)
{
    //ESP_LOGI(TAG, "micro-ROS task started");
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    
    
    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "turtlebot3_esp32_controller", "", &support));
    
    i2c_master_init();
    mpu_init();
    ak8963_read_asa(asa_adj);
    ak8963_start_16bit_100hz();
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

    // Timer (33ms -> ~30Hz)
    // Timer (20ms -> 50Hz)
    rcl_timer_t timer;
    const unsigned int timer_timeout = 20;
    RCCHECK(rclc_timer_init_default2(&timer, &support, RCL_MS_TO_NS(timer_timeout), timer_callback, true));

    // Encoder message initialization
    std_msgs__msg__Int32MultiArray__init(&encoder_msg);
    encoder_msg.data.size = 2;
    encoder_msg.data.capacity = 2;
    encoder_msg.data.data = malloc(sizeof(int32_t) * 2);

    // Executor
    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));
    RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA));   

    while(1){
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
        //usleep(10);
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
