/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Blue Pill quad flight controller
  *                   - 4 PWM receiver channels on TIM3, CH5 servo input on TIM4
  *                   - 4 ESC outputs on TIM2
  *                   - payload servo output on TIM1_CH1 / PA8
  *                   - GY-87 / MPU6050 via hardware I2C1 on PB6/PB7
  *                   - Fixed ANGLE mode with stick arming/disarming
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "telemetry.h"
#include "bmp388.h"
#include "gps.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
  float ax;
  float ay;
  float az;
  float gx;
  float gy;
  float gz;
} ImuSample;

typedef struct {
  float roll_deg;
  float pitch_deg;
  float yaw_deg;
} AttitudeState;

typedef enum {
  FAILSAFE_NONE = 0,
  FAILSAFE_HOLD,
  FAILSAFE_DESCEND,
  FAILSAFE_RECOVER
} FailsafeMode;

typedef struct {
  float kp;
  float ki;
  float kd;
  float integrator;
  float prev_measurement;
  float d_filter;
  float integrator_limit;
  float output_limit;
} PidAxis;

typedef struct {
  bool valid;
  bool rf_loss_detected;
  bool throttle_low;
  bool yaw_arm_request;
  bool yaw_disarm_request;
  float throttle_norm;
  float throttle_stick_norm;
  float roll_cmd;
  float pitch_cmd;
  float yaw_cmd;
  uint16_t raw[4];
} RcCommand;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RAD_TO_DEG                  57.29577951f
#define DEG_TO_RAD                  0.01745329252f

#define RC_CHANNELS                 4U

#define LOOP_HZ                     250U
#define LOG_PERIOD_DISARMED_MS      200U
#define LOG_PERIOD_ARMED_MS         500U

#define UART_TX_BUFFER_SIZE         1024U

#define RX_TIMEOUT_MS               120U
#define ARM_HOLD_MS                 3000U
#define DISARM_HOLD_MS              3000U

#define FAILSAFE_AIRBORNE_THROTTLE  0.18f
#define FAILSAFE_HOLD_MS            1300U
#define FAILSAFE_RECOVER_VALID_MS   400U
#define FAILSAFE_RECOVER_BLEND_MS   900U
#define FAILSAFE_DESCEND_RATE_HIGH  0.07f
#define FAILSAFE_DESCEND_RATE_LOW   0.035f
#define FAILSAFE_DESCEND_SPLIT      0.18f
#define FAILSAFE_DESCEND_MIN        0.12f
#define FAILSAFE_DESCEND_FINAL      0.07f
#define FAILSAFE_LAND_ATTITUDE_DEG  14.0f
#define FAILSAFE_LAND_RATE_DPS      35.0f
#define FAILSAFE_LAND_STABLE_MS     1200U
#define RF_PRESET_CENTER_US         1500U
#define RF_PRESET_WINDOW_US         12U
#define RF_PRESET_THROTTLE_MAX_US   950U
#define RF_PRESET_DETECT_MS         120U

#define MOTOR_STOP_US               1000U
#define MOTOR_IDLE_US               1060U
#define MOTOR_MAX_US                2000U
#define MOTOR_HEADROOM_US           260U
#define THROTTLE_LOW_US             1165U

#define RC_VALID_MIN_US             850U
#define RC_VALID_MAX_US             2200U
#define RC_DEADBAND_US              24U

/* Updated from the latest bench logs on the current airframe/radio. */
#define RC_ROLL_MIN_US              1310U
#define RC_ROLL_CENTER_US           1539U
#define RC_ROLL_MAX_US              1808U

#define RC_PITCH_MIN_US             1290U
#define RC_PITCH_CENTER_US          1525U
#define RC_PITCH_MAX_US             1789U

#define RC_YAW_MIN_US               1259U
#define RC_YAW_CENTER_US            1508U
#define RC_YAW_MAX_US               1760U

#define RC_THROTTLE_MIN_US          990U
#define RC_THROTTLE_MAX_US          2120U

#define SERVO_GRIP_US               900U
#define SERVO_OPEN_US               2600U
#define SERVO_CH5_GRIP_US           1300U
#define SERVO_CH5_OPEN_US           1700U
#define SERVO_CH5_TIMEOUT_MS        300U

#define ARM_YAW_HIGH_US             1700U
#define DISARM_YAW_LOW_US           1320U
#define ARM_STICK_CENTER_WINDOW_US  120U

#define MAX_ANGLE_DEG               13.0f
#define MAX_ANGLE_RATE_DPS          120.0f
#define MAX_YAW_RATE_DPS            140.0f
#define ANGLE_OUTER_KP              4.0f
#define CENTER_BRAKE_STICK_THRESH   0.18f
#define CENTER_BRAKE_KP_BOOST       1.6f
#define HEADING_HOLD_YAW_DEADBAND   0.05f
#define HEADING_HOLD_YAW_EXIT_DB     0.09f
#define HEADING_HOLD_KP             1.85f
#define HEADING_HOLD_RATE_LIMIT_DPS 55.0f
#define HEADING_HOLD_ERROR_LIMIT_DEG 24.0f
#define HEADING_HOLD_RELEASE_MS     120U
#define HEADING_HOLD_MIN_THROTTLE   0.10f
#define HEADING_HOLD_MAX_RP_STICK   0.18f
#define HEADING_HOLD_MAX_TILT_DEG   8.0f
#define HEADING_HOLD_MAX_RP_RATE_DPS 65.0f
#define ACCEL_CORRECTION_GAIN       2.4f
#define ACCEL_TRUST_MAX_ERROR_G     0.35f
#define MAG_YAW_CORRECTION_GAIN     0.30f
#define MAG_CORRECTION_MAX_TILT_DEG 18.0f
#define MAG_CORRECTION_MAX_RP_RATE_DPS 90.0f
#define MAG_CORRECTION_MIN_ACCEL_TRUST 0.35f
#define MAG_CORRECTION_MAX_THR_DELTA 0.040f
#define MAG_FIELD_MIN_GAUSS         0.08f
#define MAG_FIELD_MAX_GAUSS         2.50f
#define MAG_HEADING_JUMP_DEG        35.0f
#define MAG_HEALTH_RECOVER_MS       300U
#define MAG_RAW_LPF_ALPHA           0.12f
#define RC_CENTER_LEARN_ALPHA       0.010f
#define RC_CENTER_LEARN_WINDOW_US   70U
#define RC_CENTER_LEARN_MIN_MS      1200U
#define TILT_THROTTLE_MIN           0.18f
#define TILT_THROTTLE_MAX_BOOST     0.16f
#define YAW_MANEUEVER_STICK_THRESH  0.18f
#define YAW_CONTROL_MIN_THROTTLE    0.10f
#define YAW_MANEUVER_AUTHORITY_MIN  0.60f
#define YAW_INTEGRATOR_DECAY_MANEUVER 0.92f
#define YAW_AUTHORITY_FLOOR         0.68f
#define YAW_DYNAMIC_DAMPING_KP      0.70f
#define YAW_DYNAMIC_DAMPING_THR_KP  0.95f
#define YAW_DYNAMIC_DAMPING_RP_KP   0.55f
#define YAW_THROTTLE_FF            -28.0f
#define YAW_THROTTLE_HIGH_FF       -52.0f
#define YAW_THROTTLE_DELTA_FF      -3.2f
#define YAW_TAKEOFF_BIAS_FF        -9.0f
#define YAW_ROLL_COUPLING_FF        0.0f
#define YAW_PITCH_COUPLING_FF       0.0f
#define YAW_RP_MANEUVER_BIAS_FF    -32.0f
#define YAW_RP_OUTPUT_BIAS_FF      -0.105f
#define YAW_RP_RATE_BIAS_FF        -0.055f
#define YAW_RP_BIAS_THR_GAIN        1.65f
#define YAW_RP_BIAS_LIMIT           115.0f
#define YAW_MIX_SIGN                1.0f
#define YAW_SAT_INTEGRATOR_DECAY    0.94f
#define YAW_BIAS_LEARN_MIN_THROTTLE 0.30f
#define YAW_BIAS_LEARN_MAX_RATE_DPS 85.0f
#define YAW_BIAS_LEARN_MAX_CMD_DPS  18.0f
#define YAW_BIAS_LEARN_MAX_ERROR_DEG 6.0f
#define YAW_BIAS_LEARN_GAIN         2.0f
#define YAW_BIAS_DECAY              0.995f
#define YAW_BIAS_SAT_DECAY          0.980f
#define YAW_BIAS_LIMIT              130.0f
#define SENSOR_LEVEL_ROLL_OFFSET_DEG  0.0f
#define SENSOR_LEVEL_PITCH_OFFSET_DEG 0.0f
#define HOVER_ROLL_TRIM_DEG           0.0f
#define HOVER_PITCH_TRIM_DEG          0.0f
#define HOVER_TRIM_START_THROTTLE     0.12f
#define HOVER_TRIM_FULL_THROTTLE      0.35f
#define TAKEOFF_ASSIST_MIN_THROTTLE   0.16f
#define TAKEOFF_ASSIST_MAX_THROTTLE   0.62f
#define TAKEOFF_ASSIST_STICK_THRESH   0.12f
#define TAKEOFF_ASSIST_KP_BOOST       1.0f
#define TAKEOFF_ROLL_TRIM_DEG         0.0f
#define TAKEOFF_PITCH_TRIM_DEG        0.0f
#define TAKEOFF_LEFT_CG_BIAS_US       0.0f
#define STICK_EXPO                    0.78f

#define GYRO_LPF_HZ                 42.0f
#define ACCEL_LPF_HZ                10.0f
#define RC_STICK_ALPHA              0.33f
#define RC_YAW_ALPHA                0.50f
#define RC_THROTTLE_ALPHA           0.06f
#define THROTTLE_EXPO               0.42f
#define THROTTLE_SLEW_UP_PER_S      3.00f
#define THROTTLE_SLEW_DOWN_PER_S    1.50f
#define COMPLEMENTARY_ALPHA         0.985f
#define PID_D_FILTER_CUTOFF_HZ      32.0f

#define GYRO_CALIB_SAMPLES          800U
#define ACCEL_CALIB_SAMPLES         400U
#define IMU_FAIL_LIMIT              10U

#define SENSOR_ROTATION_0           0U
#define SENSOR_ROTATION_90          1U
#define SENSOR_ROTATION_180         2U
#define SENSOR_ROTATION_270         3U
#define SENSOR_ROTATION             SENSOR_ROTATION_0
#define MAG_ROTATION                SENSOR_ROTATION

#define RC_ROLL_SIGN               -1.0f
#define RC_PITCH_SIGN               1.0f
#define RC_YAW_SIGN                 1.0f
#define GYRO_Z_SIGN                 -1.0f
#define MAG_X_SIGN                  1.0f
#define MAG_Y_SIGN                  1.0f
#define MAG_Z_SIGN                  1.0f
#define MAG_OFFSET_X               -0.72f
#define MAG_OFFSET_Y                0.02f
#define MAG_OFFSET_Z                0.00f

#define MPU6050_ADDR                (0x68U << 1)
#define HMC5883L_ADDR               (0x1EU << 1)
#define MPU_REG_SMPLRT_DIV          0x19U
#define MPU_REG_CONFIG              0x1AU
#define MPU_REG_GYRO_CONFIG         0x1BU
#define MPU_REG_ACCEL_CONFIG        0x1CU
#define MPU_REG_INT_PIN_CFG         0x37U
#define MPU_REG_ACCEL_XOUT_H        0x3BU
#define MPU_REG_PWR_MGMT_1          0x6BU
#define MPU_REG_WHO_AM_I            0x75U

#define HMC_REG_CONFIG_A            0x00U
#define HMC_REG_CONFIG_B            0x01U
#define HMC_REG_MODE                0x02U
#define HMC_REG_DATA_X_MSB          0x03U

#define MAG_UPDATE_MS               20U
#define MAG_FAIL_LIMIT              10U

#define MPU_ACCEL_SCALE_4G          8192.0f
#define MPU_GYRO_SCALE_500DPS       65.5f

#define SOFT_I2C_SCL_Pin            GPIO_PIN_6
#define SOFT_I2C_SCL_GPIO_Port      GPIOB
#define SOFT_I2C_SDA_Pin            GPIO_PIN_7
#define SOFT_I2C_SDA_GPIO_Port      GPIOB

#define I2C_TIMEOUT_US              3000U
#define I2C_TIMEOUT_LONG_US         12000U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define FLASH_OPT __attribute__((optimize("Os")))

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
static volatile uint32_t rc_rise_ticks[RC_CHANNELS] = {0U, 0U, 0U, 0U};
static volatile uint16_t rc_pulse_us[RC_CHANNELS] = {1500U, 1500U, 1000U, 1500U};
static volatile uint8_t rc_edge_state[RC_CHANNELS] = {0U, 0U, 0U, 0U};
static volatile uint32_t rc_last_update_ms[RC_CHANNELS] = {0U, 0U, 0U, 0U};

static volatile uint32_t servo_ch5_rise_tick = 0U;
static volatile uint16_t servo_ch5_pulse_us = 1000U;
static volatile uint32_t servo_ch5_last_update_ms = 0U;
static volatile uint8_t servo_ch5_edge_state = 0U;
static uint16_t servo_output_us = SERVO_GRIP_US;
static bool servo_released = false;

static uint16_t motor_output_us[4] = {
  MOTOR_STOP_US, MOTOR_STOP_US, MOTOR_STOP_US, MOTOR_STOP_US
};

static AttitudeState attitude = {0.0f, 0.0f, 0.0f};

static PidAxis pid_roll  = {1.18f, 0.42f, 0.046f, 0.0f, 0.0f, 0.0f, 95.0f, 300.0f};
static PidAxis pid_pitch = {1.18f, 0.42f, 0.046f, 0.0f, 0.0f, 0.0f, 95.0f, 300.0f};
static PidAxis pid_yaw   = {2.60f, 0.55f, 0.000f, 0.0f, 0.0f, 0.0f, 120.0f, 340.0f};

static float gyro_bias_x_dps = 0.0f;
static float gyro_bias_y_dps = 0.0f;
static float gyro_bias_z_dps = 0.0f;
static float accel_roll_trim = 0.0f;
static float accel_pitch_trim = 0.0f;

static float gyro_lpf_x = 0.0f;
static float gyro_lpf_y = 0.0f;
static float gyro_lpf_z = 0.0f;
static float accel_lpf_x = 0.0f;
static float accel_lpf_y = 0.0f;
static float accel_lpf_z = 1.0f;

static float rc_roll_filtered = 0.0f;
static float rc_pitch_filtered = 0.0f;
static float rc_yaw_filtered = 0.0f;
static float rc_throttle_filtered = 0.0f;

static bool imu_ready = false;
static bool armed = false;
static bool failsafe_active = true;
static FailsafeMode failsafe_mode = FAILSAFE_NONE;
static uint8_t imu_fail_streak = 0U;
static uint32_t arm_request_ms = 0U;
static uint32_t disarm_request_ms = 0U;
static uint32_t failsafe_state_ms = 0U;
static uint32_t rf_recover_ms = 0U;
static uint32_t land_detect_ms = 0U;
static float failsafe_throttle_norm = 0.0f;
static float last_pilot_throttle_norm = 0.0f;
static bool mag_ready = false;
static bool mag_usable = false;
static bool mag_correction_allowed = false;
static bool heading_hold_engaged = false;
static bool heading_hold_allowed = false;
static bool mag_calibrated = true;
static uint8_t mag_fail_streak = 0U;
static uint32_t last_mag_update_ms = 0U;
static uint32_t last_heading_hold_ms = 0U;
static uint32_t rf_preset_detect_ms = 0U;
static uint32_t mag_good_since_ms = 0U;
static uint32_t rc_center_learn_ms = 0U;
static float throttle_command_norm = 0.0f;
static float heading_deg = 0.0f;
static float heading_hold_ref_deg = 0.0f;
static float mag_heading_deg = 0.0f;
static float mag_field_strength = 0.0f;
static float mag_field_filtered = 0.0f;
static float mag_heading_jump = 0.0f;
static float mag_offset_x = MAG_OFFSET_X;
static float mag_offset_y = MAG_OFFSET_Y;
static float mag_offset_z = MAG_OFFSET_Z;
static bool mag_calib_active = false;
static uint32_t mag_calib_start_ms = 0U;
static float mag_calib_min_x = 999.0f;
static float mag_calib_max_x = -999.0f;
static float mag_calib_min_y = 999.0f;
static float mag_calib_max_y = -999.0f;
static float mag_calib_min_z = 999.0f;
static float mag_calib_max_z = -999.0f;
static float mag_raw_x = 0.0f;
static float mag_raw_y = 0.0f;
static float mag_raw_z = 0.0f;
static float mag_lpf_x = 0.0f;
static float mag_lpf_y = 0.0f;
static float mag_lpf_z = 0.0f;
static bool mag_heading_valid = false;
static float rc_roll_center_runtime = (float)RC_ROLL_CENTER_US;
static float rc_pitch_center_runtime = (float)RC_PITCH_CENTER_US;
static float rc_yaw_center_runtime = (float)RC_YAW_CENTER_US;
static float accel_norm = 1.0f;
static float accel_trust = 0.0f;
static float throttle_tilt_boost = 0.0f;
static float takeoff_assist_scale = 0.0f;
static float last_roll_pid_output = 0.0f;
static float last_pitch_pid_output = 0.0f;
static float last_yaw_pid_output = 0.0f;
static float last_desired_yaw_rate = 0.0f;
static float last_gyro_x_dps = 0.0f;
static float last_gyro_y_dps = 0.0f;
static float last_heading_error_deg = 0.0f;
static float last_gyro_z_dps = 0.0f;
static float last_control_throttle_norm = 0.0f;
static float throttle_delta_norm = 0.0f;
static float throttle_rate_norm_s = 0.0f;
static float yaw_dynamic_damping_output = 0.0f;
static float yaw_throttle_feedforward_output = 0.0f;
static float yaw_feedforward_output = 0.0f;
static float yaw_maneuver_bias_output = 0.0f;
static float yaw_bias_output = 0.0f;
static float heading_hold_limited_error_deg = 0.0f;
static float mixer_yaw_scale = 1.0f;
static float mixer_rp_scale = 1.0f;
static float yaw_maneuver_scale = 1.0f;
static bool roll_pitch_dynamic = false;
static bool mixer_saturated = false;

static char uart_buf[768];
static volatile char uart_tx_ring[UART_TX_BUFFER_SIZE];
static volatile uint16_t uart_tx_head = 0U;
static volatile uint16_t uart_tx_tail = 0U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
static void dwt_init(void);
static uint32_t dwt_cycles32(void);
void uart_log(const char *text);
static void uart_start_tx(void);
static void set_status_led(bool on);
static void update_status_led(void);

static float clampf(float value, float min_value, float max_value);
static float apply_expo(float value, float expo);
static float lpf_alpha(float cutoff_hz, float dt);
static float lpf_apply(float previous, float input, float alpha);
static uint16_t pulse_distance_us(uint16_t a, uint16_t b);
static float normalize_centered(uint16_t pulse_us,
                                uint16_t min_us,
                                float center_us,
                                uint16_t max_us,
                                float sign);
static float normalize_throttle(uint16_t pulse_us);
static float apply_throttle_curve(float value);
static void rotate_xy(float *x, float *y);
static void rotate_mag_axes(float *x, float *y, float *z);
static void update_rc_center_learning(RcCommand *command, uint32_t now_ms);
static float control_roll_accel_deg(const ImuSample *sample);
static float control_pitch_accel_deg(const ImuSample *sample);
static float control_roll_rate_dps(const ImuSample *sample);
static float control_pitch_rate_dps(const ImuSample *sample);
static float control_yaw_rate_dps(const ImuSample *sample);
static float hover_trim_scale(float throttle_norm);

static void pid_reset(PidAxis *pid);
static void reset_all_pids(void);
static float pid_update(PidAxis *pid, float setpoint, float measurement, float dt, bool enable_integrator);

static void hw_i2c1_init(void);
static bool hw_i2c_wait_flag(volatile uint32_t *reg, uint32_t flag, bool set, uint32_t timeout_us);
bool hw_i2c_write_register(uint8_t device_addr, uint8_t reg, uint8_t value);
bool hw_i2c_read_registers(uint8_t device_addr, uint8_t reg, uint8_t *buffer, uint16_t length);
bool hw_i2c_ping(uint8_t addr_7bit);
static bool imu_write_reg(uint8_t reg, uint8_t value);
static bool imu_read_regs(uint8_t reg, uint8_t *buffer, uint16_t length);
static bool mpu6050_init(void);
static bool mpu6050_enable_bypass(void);
static bool mpu6050_read_raw(ImuSample *sample);
static bool mpu6050_read_filtered(ImuSample *sample, float dt);
static bool mpu6050_calibrate(void);
static void run_level_calibration(void);
static void update_stick_commands(const RcCommand *command, uint32_t now_ms);
static void update_attitude(const ImuSample *sample, float dt);

static bool receiver_read_command(RcCommand *command, uint32_t now_ms);
static void servo_set_us(uint16_t pulse_us);
static void servo_grip(void);
static void servo_open(void);
static void servo_update_from_ch5(uint32_t now_ms);
static bool hmc5883l_init(void);
static bool hmc5883l_read(float *mx, float *my, float *mz);
static float wrap_heading_deg(float heading_deg_value);
static float heading_error_deg(float target_deg, float current_deg);
static float compute_heading_deg(float mx, float my, float mz, float pitch_deg, float roll_deg);
static void update_aux_sensors(uint32_t now_ms);

static bool detect_rf_loss_pattern(const RcCommand *command);
static bool command_is_centered(const RcCommand *command);
static void update_failsafe_state(const RcCommand *command,
                                  bool sensors_ok,
                                  const ImuSample *sample,
                                  bool sample_valid,
                                  uint32_t now_ms,
                                  float dt);
static void update_arming_state(const RcCommand *command, bool sensors_ok, uint32_t now_ms);
static float apply_throttle_slew(float target_norm, float dt, bool reset_state);
static void build_control_command(const RcCommand *pilot_command,
                                  RcCommand *control_command,
                                  uint32_t now_ms,
                                  float dt);
static void enter_rf_failsafe(uint32_t now_ms);

static void motor_set_us(uint8_t motor_index, uint16_t pulse_us);
static void motors_write_all(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4);
static void stop_all_motors(void);
static void mix_motor_outputs(float throttle_base,
                              float roll_output,
                              float pitch_output,
                              float yaw_output,
                              float left_cg_bias_us);
static void run_flight_control(const RcCommand *command, const ImuSample *sample, float dt);
static void log_status(const RcCommand *command);
static uint32_t last_baro_sample_ms = 0U;
static Bmp388Data current_baro_data = {0};

static void telemetry_fill_cb(TelemetryData *out, const void *ctx);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void dwt_init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static uint32_t dwt_cycles32(void)
{
  return DWT->CYCCNT;
}

void uart_log(const char *text)
{
  __disable_irq();

  while (*text != '\0') {
    const uint16_t next_head = (uint16_t)((uart_tx_head + 1U) % UART_TX_BUFFER_SIZE);

    if (next_head == uart_tx_tail) {
      break;
    }

    uart_tx_ring[uart_tx_head] = *text;
    uart_tx_head = next_head;
    text++;
  }

  __enable_irq();
  uart_start_tx();
}

static void uart_start_tx(void)
{
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_TXE);
}

void uart1_irq_handler(void)
{
  if ((__HAL_UART_GET_IT_SOURCE(&huart1, UART_IT_TXE) != RESET) &&
      (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TXE) != RESET)) {
    if (uart_tx_tail != uart_tx_head) {
      huart1.Instance->DR = (uint16_t)(uint8_t)uart_tx_ring[uart_tx_tail];
      uart_tx_tail = (uint16_t)((uart_tx_tail + 1U) % UART_TX_BUFFER_SIZE);
    } else {
      __HAL_UART_DISABLE_IT(&huart1, UART_IT_TXE);
    }
  }
}

static void set_status_led(bool on)
{
  HAL_GPIO_WritePin(STATUS_LED_GPIO_Port,
                    STATUS_LED_Pin,
                    on ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static void update_status_led(void)
{
  static uint32_t last_toggle_ms = 0U;
  static bool led_on = false;
  const uint32_t now_ms = HAL_GetTick();
  uint32_t period_ms = 500U;

  if (mag_calib_active) {
    period_ms = 50U;
  } else if (!imu_ready) {
    period_ms = 120U;
  } else if (failsafe_active) {
    period_ms = 100U;
  } else if (!armed) {
    period_ms = 450U;
  } else {
    set_status_led(true);
    return;
  }

  if ((uint32_t)(now_ms - last_toggle_ms) >= period_ms) {
    last_toggle_ms = now_ms;
    led_on = !led_on;
    set_status_led(led_on);
  }
}

static void hw_i2c1_init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_I2C1_CLK_ENABLE();

  GPIO_InitStruct.Pin = SOFT_I2C_SCL_Pin | SOFT_I2C_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  HAL_GPIO_WritePin(GPIOB, SOFT_I2C_SCL_Pin | SOFT_I2C_SDA_Pin, GPIO_PIN_SET);
  for (uint8_t i = 0U; i < 9U; i++) {
    HAL_GPIO_WritePin(GPIOB, SOFT_I2C_SCL_Pin, GPIO_PIN_RESET);
    for (volatile uint32_t delay = 0U; delay < 80U; delay++) { __NOP(); }
    HAL_GPIO_WritePin(GPIOB, SOFT_I2C_SCL_Pin, GPIO_PIN_SET);
    for (volatile uint32_t delay = 0U; delay < 80U; delay++) { __NOP(); }
  }
  HAL_GPIO_WritePin(GPIOB, SOFT_I2C_SDA_Pin, GPIO_PIN_RESET);
  for (volatile uint32_t delay = 0U; delay < 80U; delay++) { __NOP(); }
  HAL_GPIO_WritePin(GPIOB, SOFT_I2C_SCL_Pin, GPIO_PIN_SET);
  for (volatile uint32_t delay = 0U; delay < 80U; delay++) { __NOP(); }
  HAL_GPIO_WritePin(GPIOB, SOFT_I2C_SDA_Pin, GPIO_PIN_SET);

  GPIO_InitStruct.Pin = SOFT_I2C_SCL_Pin | SOFT_I2C_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  __HAL_RCC_I2C1_FORCE_RESET();
  __HAL_RCC_I2C1_RELEASE_RESET();

  I2C1->CR1 = 0U;
  I2C1->CR2 = 36U;
  I2C1->CCR = 180U;
  I2C1->TRISE = 37U;
  I2C1->CR1 = I2C_CR1_PE;
}

static bool hw_i2c_wait_flag(volatile uint32_t *reg, uint32_t flag, bool set, uint32_t timeout_us)
{
  const uint32_t start = dwt_cycles32();
  const uint32_t timeout_cycles = timeout_us * (SystemCoreClock / 1000000U);

  while (((*reg & flag) != 0U) != set) {
    if ((uint32_t)(dwt_cycles32() - start) > timeout_cycles) {
      return false;
    }
  }
  return true;
}

bool hw_i2c_write_register(uint8_t device_addr, uint8_t reg, uint8_t value)
{
  if (!hw_i2c_wait_flag(&I2C1->SR2, I2C_SR2_BUSY, false, I2C_TIMEOUT_LONG_US)) {
    hw_i2c1_init();
    return false;
  }

  I2C1->CR1 |= I2C_CR1_START;
  if (!hw_i2c_wait_flag(&I2C1->SR1, I2C_SR1_SB, true, I2C_TIMEOUT_US)) {
    hw_i2c1_init();
    return false;
  }
  I2C1->DR = device_addr & 0xFEU;
  if (!hw_i2c_wait_flag(&I2C1->SR1, I2C_SR1_ADDR, true, I2C_TIMEOUT_US)) {
    I2C1->CR1 |= I2C_CR1_STOP;
    hw_i2c1_init();
    return false;
  }
  (void)I2C1->SR1;
  (void)I2C1->SR2;

  if (!hw_i2c_wait_flag(&I2C1->SR1, I2C_SR1_TXE, true, I2C_TIMEOUT_US)) {
    I2C1->CR1 |= I2C_CR1_STOP;
    return false;
  }
  I2C1->DR = reg;
  if (!hw_i2c_wait_flag(&I2C1->SR1, I2C_SR1_BTF, true, I2C_TIMEOUT_US)) {
    I2C1->CR1 |= I2C_CR1_STOP;
    return false;
  }
  I2C1->DR = value;
  if (!hw_i2c_wait_flag(&I2C1->SR1, I2C_SR1_BTF, true, I2C_TIMEOUT_US)) {
    I2C1->CR1 |= I2C_CR1_STOP;
    return false;
  }

  I2C1->CR1 |= I2C_CR1_STOP;
  return true;
}

bool hw_i2c_read_registers(uint8_t device_addr, uint8_t reg, uint8_t *buffer, uint16_t length)
{
  if (length == 0U) {
    return true;
  }

  if (!hw_i2c_wait_flag(&I2C1->SR2, I2C_SR2_BUSY, false, I2C_TIMEOUT_LONG_US)) {
    hw_i2c1_init();
    return false;
  }

  I2C1->CR1 |= I2C_CR1_ACK;
  I2C1->CR1 |= I2C_CR1_START;
  if (!hw_i2c_wait_flag(&I2C1->SR1, I2C_SR1_SB, true, I2C_TIMEOUT_US)) {
    hw_i2c1_init();
    return false;
  }
  I2C1->DR = device_addr & 0xFEU;
  if (!hw_i2c_wait_flag(&I2C1->SR1, I2C_SR1_ADDR, true, I2C_TIMEOUT_US)) {
    I2C1->CR1 |= I2C_CR1_STOP;
    hw_i2c1_init();
    return false;
  }
  (void)I2C1->SR1;
  (void)I2C1->SR2;

  if (!hw_i2c_wait_flag(&I2C1->SR1, I2C_SR1_TXE, true, I2C_TIMEOUT_US)) {
    I2C1->CR1 |= I2C_CR1_STOP;
    return false;
  }
  I2C1->DR = reg;
  if (!hw_i2c_wait_flag(&I2C1->SR1, I2C_SR1_BTF, true, I2C_TIMEOUT_US)) {
    I2C1->CR1 |= I2C_CR1_STOP;
    return false;
  }

  I2C1->CR1 |= I2C_CR1_START;
  if (!hw_i2c_wait_flag(&I2C1->SR1, I2C_SR1_SB, true, I2C_TIMEOUT_US)) {
    hw_i2c1_init();
    return false;
  }
  I2C1->DR = device_addr | 0x01U;

  if (length == 1U) {
    if (!hw_i2c_wait_flag(&I2C1->SR1, I2C_SR1_ADDR, true, I2C_TIMEOUT_US)) {
      I2C1->CR1 |= I2C_CR1_STOP;
      return false;
    }
    I2C1->CR1 &= ~I2C_CR1_ACK;
    __disable_irq();
    (void)I2C1->SR1;
    (void)I2C1->SR2;
    I2C1->CR1 |= I2C_CR1_STOP;
    __enable_irq();
    if (!hw_i2c_wait_flag(&I2C1->SR1, I2C_SR1_RXNE, true, I2C_TIMEOUT_US)) {
      return false;
    }
    buffer[0] = (uint8_t)I2C1->DR;
    I2C1->CR1 |= I2C_CR1_ACK;
    return true;
  }

  if (length == 2U) {
    I2C1->CR1 |= I2C_CR1_POS;
    if (!hw_i2c_wait_flag(&I2C1->SR1, I2C_SR1_ADDR, true, I2C_TIMEOUT_US)) {
      I2C1->CR1 |= I2C_CR1_STOP;
      I2C1->CR1 &= ~I2C_CR1_POS;
      return false;
    }
    I2C1->CR1 &= ~I2C_CR1_ACK;
    __disable_irq();
    (void)I2C1->SR1;
    (void)I2C1->SR2;
    __enable_irq();
    if (!hw_i2c_wait_flag(&I2C1->SR1, I2C_SR1_BTF, true, I2C_TIMEOUT_US)) {
      I2C1->CR1 &= ~I2C_CR1_POS;
      return false;
    }
    __disable_irq();
    I2C1->CR1 |= I2C_CR1_STOP;
    buffer[0] = (uint8_t)I2C1->DR;
    __enable_irq();
    buffer[1] = (uint8_t)I2C1->DR;
    I2C1->CR1 &= ~I2C_CR1_POS;
    I2C1->CR1 |= I2C_CR1_ACK;
    return true;
  }

  if (!hw_i2c_wait_flag(&I2C1->SR1, I2C_SR1_ADDR, true, I2C_TIMEOUT_US)) {
    I2C1->CR1 |= I2C_CR1_STOP;
    return false;
  }
  (void)I2C1->SR1;
  (void)I2C1->SR2;

  while (length > 3U) {
    if (!hw_i2c_wait_flag(&I2C1->SR1, I2C_SR1_RXNE, true, I2C_TIMEOUT_US)) {
      I2C1->CR1 |= I2C_CR1_STOP;
      return false;
    }
    *buffer++ = (uint8_t)I2C1->DR;
    length--;
  }

  if (!hw_i2c_wait_flag(&I2C1->SR1, I2C_SR1_BTF, true, I2C_TIMEOUT_US)) {
    I2C1->CR1 |= I2C_CR1_STOP;
    return false;
  }
  I2C1->CR1 &= ~I2C_CR1_ACK;
  __disable_irq();
  *buffer++ = (uint8_t)I2C1->DR;
  I2C1->CR1 |= I2C_CR1_STOP;
  __enable_irq();
  *buffer++ = (uint8_t)I2C1->DR;
  if (!hw_i2c_wait_flag(&I2C1->SR1, I2C_SR1_RXNE, true, I2C_TIMEOUT_US)) {
    return false;
  }
  *buffer = (uint8_t)I2C1->DR;
  I2C1->CR1 |= I2C_CR1_ACK;
  return true;
}

bool hw_i2c_ping(uint8_t addr_7bit)
{
  const uint32_t timeout_cycles = 1500U * (SystemCoreClock / 1000000U);
  uint32_t start;
  const uint8_t addr_8bit = (uint8_t)(addr_7bit << 1);

  start = dwt_cycles32();
  while ((I2C1->SR2 & I2C_SR2_BUSY) != 0U) {
    if ((uint32_t)(dwt_cycles32() - start) > timeout_cycles) {
      hw_i2c1_init();
      return false;
    }
  }

  I2C1->CR1 |= I2C_CR1_START;
  start = dwt_cycles32();
  while ((I2C1->SR1 & I2C_SR1_SB) == 0U) {
    if ((uint32_t)(dwt_cycles32() - start) > timeout_cycles) {
      hw_i2c1_init();
      return false;
    }
  }

  I2C1->DR = addr_8bit;
  start = dwt_cycles32();
  while (1) {
    const uint32_t sr1 = I2C1->SR1;
    if ((sr1 & I2C_SR1_ADDR) != 0U) {
      (void)I2C1->SR1;
      (void)I2C1->SR2;
      I2C1->CR1 |= I2C_CR1_STOP;
      start = dwt_cycles32();
      while ((I2C1->CR1 & I2C_CR1_STOP) != 0U) {
        if ((uint32_t)(dwt_cycles32() - start) > timeout_cycles) {
          break;
        }
      }
      return true;
    }
    if ((sr1 & I2C_SR1_AF) != 0U) {
      I2C1->SR1 &= ~I2C_SR1_AF;
      I2C1->CR1 |= I2C_CR1_STOP;
      start = dwt_cycles32();
      while ((I2C1->CR1 & I2C_CR1_STOP) != 0U) {
        if ((uint32_t)(dwt_cycles32() - start) > timeout_cycles) {
          break;
        }
      }
      return false;
    }
    if ((uint32_t)(dwt_cycles32() - start) > timeout_cycles) {
      I2C1->CR1 |= I2C_CR1_STOP;
      hw_i2c1_init();
      return false;
    }
  }
}

static bool hmc5883l_init(void)
{
  return hw_i2c_write_register(HMC5883L_ADDR, HMC_REG_CONFIG_A, 0x70U) &&
         hw_i2c_write_register(HMC5883L_ADDR, HMC_REG_CONFIG_B, 0x20U) &&
         hw_i2c_write_register(HMC5883L_ADDR, HMC_REG_MODE, 0x00U);
}

static bool hmc5883l_read(float *mx, float *my, float *mz)
{
  uint8_t buf[6];

  if (!hw_i2c_read_registers(HMC5883L_ADDR, HMC_REG_DATA_X_MSB, buf, sizeof(buf))) {
    return false;
  }

  *mx = (float)(int16_t)((buf[0] << 8) | buf[1]) / 1090.0f;
  *mz = (float)(int16_t)((buf[2] << 8) | buf[3]) / 1090.0f;
  *my = (float)(int16_t)((buf[4] << 8) | buf[5]) / 1090.0f;
  return true;
}

static float wrap_heading_deg(float heading_deg_value)
{
  while (heading_deg_value >= 360.0f) {
    heading_deg_value -= 360.0f;
  }
  while (heading_deg_value < 0.0f) {
    heading_deg_value += 360.0f;
  }
  return heading_deg_value;
}

static float heading_error_deg(float target_deg, float current_deg)
{
  float error = wrap_heading_deg(target_deg) - wrap_heading_deg(current_deg);

  while (error > 180.0f) {
    error -= 360.0f;
  }
  while (error < -180.0f) {
    error += 360.0f;
  }

  return error;
}

static float compute_heading_deg(float mx, float my, float mz, float pitch_deg, float roll_deg)
{
  const float pitch = pitch_deg * DEG_TO_RAD;
  const float roll = roll_deg * DEG_TO_RAD;
  const float mx2 = (mx * cosf(pitch)) + (mz * sinf(pitch));
  const float my2 = (mx * sinf(roll) * sinf(pitch)) +
                    (my * cosf(roll)) -
                    (mz * sinf(roll) * cosf(pitch));
  float yaw = atan2f(-my2, mx2) * RAD_TO_DEG;

  return wrap_heading_deg(yaw);
}

static bool detect_rf_loss_pattern(const RcCommand *command)
{
  return (command->raw[2] <= RF_PRESET_THROTTLE_MAX_US) &&
         (pulse_distance_us(command->raw[0], RF_PRESET_CENTER_US) <= RF_PRESET_WINDOW_US) &&
         (pulse_distance_us(command->raw[1], RF_PRESET_CENTER_US) <= RF_PRESET_WINDOW_US) &&
         (pulse_distance_us(command->raw[3], RF_PRESET_CENTER_US) <= RF_PRESET_WINDOW_US);
}

static void FLASH_OPT update_aux_sensors(uint32_t now_ms)
{
  if ((uint32_t)(now_ms - last_mag_update_ms) >= MAG_UPDATE_MS) {
    float mx;
    float my;
    float mz;
    float heading_source_x;
    float heading_source_y;
    float heading_source_z;

    last_mag_update_ms = now_ms;

    if (mag_calib_active) {
      if (hmc5883l_read(&mx, &my, &mz)) {
        if (mx < mag_calib_min_x) { mag_calib_min_x = mx; }
        if (mx > mag_calib_max_x) { mag_calib_max_x = mx; }
        if (my < mag_calib_min_y) { mag_calib_min_y = my; }
        if (my > mag_calib_max_y) { mag_calib_max_y = my; }
        if (mz < mag_calib_min_z) { mag_calib_min_z = mz; }
        if (mz > mag_calib_max_z) { mag_calib_max_z = mz; }
        mag_raw_x = mx;
        mag_raw_y = my;
        mag_raw_z = mz;
      }

      if ((uint32_t)(now_ms - mag_calib_start_ms) >= 20000U) {
        mag_calib_active = false;
        if (((mag_calib_max_x - mag_calib_min_x) > 0.10f) &&
            ((mag_calib_max_y - mag_calib_min_y) > 0.10f)) {
          mag_offset_x = (mag_calib_max_x + mag_calib_min_x) * 0.5f;
          mag_offset_y = (mag_calib_max_y + mag_calib_min_y) * 0.5f;
          mag_offset_z = (mag_calib_max_z + mag_calib_min_z) * 0.5f;
          mag_calibrated = true;
          (void)snprintf(uart_buf, sizeof(uart_buf),
                         "[MAG] Calibration DONE! Offsets: X=%.2f Y=%.2f Z=%.2f (range X=%.2f Y=%.2f Z=%.2f)\r\n",
                         mag_offset_x, mag_offset_y, mag_offset_z,
                         mag_calib_max_x - mag_calib_min_x,
                         mag_calib_max_y - mag_calib_min_y,
                         mag_calib_max_z - mag_calib_min_z);
          uart_log(uart_buf);
        } else {
          uart_log("[MAG] Calibration FAILED: insufficient rotation range!\r\n");
        }
      }
      return;
    }

    if (hmc5883l_read(&mx, &my, &mz)) {
      float heading_error_to_estimate;
      bool heading_jump_ok = true;

      if ((mag_lpf_x == 0.0f) && (mag_lpf_y == 0.0f) && (mag_lpf_z == 0.0f)) {
        mag_lpf_x = mx;
        mag_lpf_y = my;
        mag_lpf_z = mz;
      } else {
        mag_lpf_x = lpf_apply(mag_lpf_x, mx, MAG_RAW_LPF_ALPHA);
        mag_lpf_y = lpf_apply(mag_lpf_y, my, MAG_RAW_LPF_ALPHA);
        mag_lpf_z = lpf_apply(mag_lpf_z, mz, MAG_RAW_LPF_ALPHA);
      }

      heading_source_x = mag_lpf_x - mag_offset_x;
      heading_source_y = mag_lpf_y - mag_offset_y;
      heading_source_z = mag_lpf_z - mag_offset_z;
      rotate_mag_axes(&heading_source_x, &heading_source_y, &heading_source_z);

      mag_field_strength = sqrtf((heading_source_x * heading_source_x) +
                                 (heading_source_y * heading_source_y) +
                                 (heading_source_z * heading_source_z));
      if (mag_field_filtered <= 0.0f) {
        mag_field_filtered = mag_field_strength;
      } else {
        mag_field_filtered = lpf_apply(mag_field_filtered, mag_field_strength, 0.08f);
      }

      const float computed_heading = compute_heading_deg(heading_source_x,
                                                         heading_source_y,
                                                         heading_source_z,
                                                         attitude.pitch_deg,
                                                         attitude.roll_deg);

      mag_heading_jump = mag_heading_valid ? fabsf(heading_error_deg(computed_heading, mag_heading_deg)) : 0.0f;
      heading_jump_ok = (!mag_heading_valid) || (mag_heading_jump <= MAG_HEADING_JUMP_DEG);

      mag_fail_streak = 0U;
      mag_ready = true;
      mag_raw_x = mag_lpf_x;
      mag_raw_y = mag_lpf_y;
      mag_raw_z = mag_lpf_z;
      mag_heading_deg = computed_heading;

      if ((mag_field_strength >= MAG_FIELD_MIN_GAUSS) &&
          (mag_field_strength <= MAG_FIELD_MAX_GAUSS) &&
          heading_jump_ok &&
          mag_calibrated) {
        if (mag_good_since_ms == 0U) {
          mag_good_since_ms = now_ms;
        }
        mag_usable = ((uint32_t)(now_ms - mag_good_since_ms) >= MAG_HEALTH_RECOVER_MS);
      } else {
        mag_good_since_ms = 0U;
        mag_usable = false;
      }

      mag_correction_allowed = mag_usable &&
                               (fabsf(attitude.roll_deg) <= MAG_CORRECTION_MAX_TILT_DEG) &&
                               (fabsf(attitude.pitch_deg) <= MAG_CORRECTION_MAX_TILT_DEG) &&
                               (fabsf(gyro_lpf_x) <= MAG_CORRECTION_MAX_RP_RATE_DPS) &&
                               (fabsf(gyro_lpf_y) <= MAG_CORRECTION_MAX_RP_RATE_DPS) &&
                               (accel_trust >= MAG_CORRECTION_MIN_ACCEL_TRUST) &&
                               (fabsf(throttle_delta_norm) <= MAG_CORRECTION_MAX_THR_DELTA);

      if (mag_correction_allowed) {
        heading_error_to_estimate = heading_error_deg(computed_heading, attitude.yaw_deg);
        if (!armed && !heading_hold_engaged) {
          attitude.yaw_deg = computed_heading;
        } else {
          attitude.yaw_deg = wrap_heading_deg(attitude.yaw_deg +
                                              (heading_error_to_estimate *
                                               clampf(MAG_YAW_CORRECTION_GAIN * ((float)MAG_UPDATE_MS / 1000.0f),
                                                      0.0f,
                                                      0.04f)));
        }
        heading_deg = attitude.yaw_deg;
      }

      mag_heading_valid = true;
    } else if (++mag_fail_streak >= MAG_FAIL_LIMIT) {
      mag_ready = false;
      mag_usable = false;
      mag_correction_allowed = false;
      mag_good_since_ms = 0U;
    }
  }

}

static float apply_throttle_slew(float target_norm, float dt, bool reset_state)
{
  const float rising_step = THROTTLE_SLEW_UP_PER_S * dt;
  const float falling_step = THROTTLE_SLEW_DOWN_PER_S * dt;

  if (reset_state) {
    throttle_command_norm = clampf(target_norm, 0.0f, 1.0f);
    return throttle_command_norm;
  }

  if (target_norm > throttle_command_norm) {
    throttle_command_norm = fminf(target_norm, throttle_command_norm + rising_step);
  } else {
    throttle_command_norm = fmaxf(target_norm, throttle_command_norm - falling_step);
  }

  return throttle_command_norm;
}

static float clampf(float value, float min_value, float max_value)
{
  if (value < min_value) {
    return min_value;
  }
  if (value > max_value) {
    return max_value;
  }
  return value;
}

static float apply_expo(float value, float expo)
{
  return (value * (1.0f - expo)) + (value * value * value * expo);
}

static float lpf_alpha(float cutoff_hz, float dt)
{
  const float rc = 1.0f / (2.0f * 3.14159265f * cutoff_hz);
  return clampf(dt / (rc + dt), 0.0f, 1.0f);
}

static float lpf_apply(float previous, float input, float alpha)
{
  return previous + alpha * (input - previous);
}

static uint16_t pulse_distance_us(uint16_t a, uint16_t b)
{
  return (a >= b) ? (a - b) : (b - a);
}

static float normalize_centered(uint16_t pulse_us,
                                uint16_t min_us,
                                float center_us,
                                uint16_t max_us,
                                float sign)
{
  float centered = (float)pulse_us - center_us;
  float span;

  if (fabsf(centered) < (float)RC_DEADBAND_US) {
    centered = 0.0f;
  } else if (centered > 0.0f) {
    centered -= (float)RC_DEADBAND_US;
  } else {
    centered += (float)RC_DEADBAND_US;
  }

  if (centered >= 0.0f) {
    span = (float)max_us - center_us - (float)RC_DEADBAND_US;
  } else {
    span = center_us - (float)min_us - (float)RC_DEADBAND_US;
  }

  if (span < 1.0f) {
    span = 1.0f;
  }

  centered /= span;
  centered = clampf(centered, -1.0f, 1.0f);

  return apply_expo(centered, STICK_EXPO) * sign;
}

static void update_rc_center_learning(RcCommand *command, uint32_t now_ms)
{
  const bool sticks_near_center =
    (fabsf((float)command->raw[0] - rc_roll_center_runtime) <= (float)RC_CENTER_LEARN_WINDOW_US) &&
    (fabsf((float)command->raw[1] - rc_pitch_center_runtime) <= (float)RC_CENTER_LEARN_WINDOW_US);

  if (armed || !command->valid || !command->throttle_low || !sticks_near_center ||
      command->yaw_arm_request || command->yaw_disarm_request) {
    rc_center_learn_ms = 0U;
    return;
  }

  if (rc_center_learn_ms == 0U) {
    rc_center_learn_ms = now_ms;
    return;
  }

  if ((uint32_t)(now_ms - rc_center_learn_ms) < RC_CENTER_LEARN_MIN_MS) {
    return;
  }

  rc_roll_center_runtime = lpf_apply(rc_roll_center_runtime, (float)command->raw[0], RC_CENTER_LEARN_ALPHA);
  rc_pitch_center_runtime = lpf_apply(rc_pitch_center_runtime, (float)command->raw[1], RC_CENTER_LEARN_ALPHA);
  rc_yaw_center_runtime = (float)RC_YAW_CENTER_US;
}

static float normalize_throttle(uint16_t pulse_us)
{
  return clampf(((float)pulse_us - (float)RC_THROTTLE_MIN_US) /
                (float)(RC_THROTTLE_MAX_US - RC_THROTTLE_MIN_US),
                0.0f,
                1.0f);
}

static float apply_throttle_curve(float value)
{
  const float clamped = clampf(value, 0.0f, 1.0f);

  return ((1.0f - THROTTLE_EXPO) * clamped) + (THROTTLE_EXPO * clamped * clamped);
}

static void rotate_xy(float *x, float *y)
{
  const float original_x = *x;
  const float original_y = *y;

  switch (SENSOR_ROTATION) {
    case SENSOR_ROTATION_90:
      *x = -original_y;
      *y = original_x;
      break;

    case SENSOR_ROTATION_180:
      *x = -original_x;
      *y = -original_y;
      break;

    case SENSOR_ROTATION_270:
      *x = original_y;
      *y = -original_x;
      break;

    case SENSOR_ROTATION_0:
    default:
      break;
  }
}

static void rotate_mag_axes(float *x, float *y, float *z)
{
  const float original_x = *x * MAG_X_SIGN;
  const float original_y = *y * MAG_Y_SIGN;

  *z *= MAG_Z_SIGN;

  switch (MAG_ROTATION) {
    case SENSOR_ROTATION_90:
      *x = -original_y;
      *y = original_x;
      break;

    case SENSOR_ROTATION_180:
      *x = -original_x;
      *y = -original_y;
      break;

    case SENSOR_ROTATION_270:
      *x = original_y;
      *y = -original_x;
      break;

    case SENSOR_ROTATION_0:
    default:
      *x = original_x;
      *y = original_y;
      break;
  }
}

static float control_roll_accel_deg(const ImuSample *sample)
{
  return atan2f(-sample->ax,
                sqrtf(sample->ay * sample->ay + sample->az * sample->az)) * RAD_TO_DEG;
}

static float control_pitch_accel_deg(const ImuSample *sample)
{
  return atan2f(sample->ay, sample->az) * RAD_TO_DEG;
}

static float control_roll_rate_dps(const ImuSample *sample)
{
  return sample->gy;
}

static float control_pitch_rate_dps(const ImuSample *sample)
{
  return sample->gx;
}

static float control_yaw_rate_dps(const ImuSample *sample)
{
  return sample->gz;
}

static float hover_trim_scale(float throttle_norm)
{
  if (throttle_norm <= HOVER_TRIM_START_THROTTLE) {
    return 0.0f;
  }

  return clampf((throttle_norm - HOVER_TRIM_START_THROTTLE) /
                (HOVER_TRIM_FULL_THROTTLE - HOVER_TRIM_START_THROTTLE),
                0.0f,
                1.0f);
}

static void pid_reset(PidAxis *pid)
{
  pid->integrator = 0.0f;
  pid->prev_measurement = 0.0f;
  pid->d_filter = 0.0f;
}

static void reset_all_pids(void)
{
  pid_reset(&pid_roll);
  pid_reset(&pid_pitch);
  pid_reset(&pid_yaw);
  yaw_bias_output = 0.0f;
}

static float pid_update(PidAxis *pid, float setpoint, float measurement, float dt, bool enable_integrator)
{
  const float error = setpoint - measurement;
  const float safe_dt = fmaxf(dt, 0.0005f);
  const float derivative = (pid->prev_measurement - measurement) / safe_dt;
  const float d_alpha = lpf_alpha(PID_D_FILTER_CUTOFF_HZ, safe_dt);
  float candidate_integrator = pid->integrator;
  float output;

  pid->prev_measurement = measurement;
  pid->d_filter += d_alpha * (derivative - pid->d_filter);

  if (enable_integrator) {
    candidate_integrator += pid->ki * error * safe_dt;
    candidate_integrator = clampf(candidate_integrator,
                                  -pid->integrator_limit,
                                  pid->integrator_limit);
  } else {
    candidate_integrator *= 0.80f;
  }

  output = (pid->kp * error) + candidate_integrator + (pid->kd * pid->d_filter);

  if ((fabsf(output) < pid->output_limit) ||
      ((output > pid->output_limit) && (error < 0.0f)) ||
      ((output < -pid->output_limit) && (error > 0.0f))) {
    pid->integrator = candidate_integrator;
  }

  output = (pid->kp * error) + pid->integrator + (pid->kd * pid->d_filter);
  return clampf(output, -pid->output_limit, pid->output_limit);
}

static bool imu_write_reg(uint8_t reg, uint8_t value)
{
  for (uint8_t attempt = 0U; attempt < 3U; attempt++) {
    if (hw_i2c_write_register(MPU6050_ADDR, reg, value)) {
      return true;
    }
    hw_i2c1_init();
  }
  return false;
}

static bool imu_read_regs(uint8_t reg, uint8_t *buffer, uint16_t length)
{
  for (uint8_t attempt = 0U; attempt < 3U; attempt++) {
    if (hw_i2c_read_registers(MPU6050_ADDR, reg, buffer, length)) {
      return true;
    }
    hw_i2c1_init();
  }
  return false;
}

static bool mpu6050_init(void)
{
  uint8_t who_am_i = 0U;

  if (!imu_read_regs(MPU_REG_WHO_AM_I, &who_am_i, 1U)) {
    return false;
  }

  if ((who_am_i != 0x68U) && (who_am_i != 0x70U)) {
    return false;
  }

  if (!imu_write_reg(MPU_REG_PWR_MGMT_1, 0x00U)) {
    return false;
  }
  HAL_Delay(100U);

  if (!imu_write_reg(MPU_REG_INT_PIN_CFG, 0x02U)) {
    return false;
  }

  if (!imu_write_reg(MPU_REG_SMPLRT_DIV, 0x03U)) {
    return false;
  }
  if (!imu_write_reg(MPU_REG_CONFIG, 0x03U)) {
    return false;
  }
  if (!imu_write_reg(MPU_REG_GYRO_CONFIG, 0x08U)) {
    return false;
  }
  if (!imu_write_reg(MPU_REG_ACCEL_CONFIG, 0x08U)) {
    return false;
  }

  return true;
}

static bool mpu6050_enable_bypass(void)
{
  return imu_write_reg(MPU_REG_INT_PIN_CFG, 0x02U);
}

static bool mpu6050_read_raw(ImuSample *sample)
{
  uint8_t raw[14];
  int16_t accel_x_raw;
  int16_t accel_y_raw;
  int16_t accel_z_raw;
  int16_t gyro_x_raw;
  int16_t gyro_y_raw;
  int16_t gyro_z_raw;

  if (!imu_read_regs(MPU_REG_ACCEL_XOUT_H, raw, sizeof(raw))) {
    return false;
  }

  accel_x_raw = (int16_t)((raw[0] << 8) | raw[1]);
  accel_y_raw = (int16_t)((raw[2] << 8) | raw[3]);
  accel_z_raw = (int16_t)((raw[4] << 8) | raw[5]);

  gyro_x_raw = (int16_t)((raw[8] << 8) | raw[9]);
  gyro_y_raw = (int16_t)((raw[10] << 8) | raw[11]);
  gyro_z_raw = (int16_t)((raw[12] << 8) | raw[13]);

  sample->ax = (float)accel_x_raw / MPU_ACCEL_SCALE_4G;
  sample->ay = (float)accel_y_raw / MPU_ACCEL_SCALE_4G;
  sample->az = (float)accel_z_raw / MPU_ACCEL_SCALE_4G;

  sample->gx = ((float)gyro_x_raw / MPU_GYRO_SCALE_500DPS) - gyro_bias_x_dps;
  sample->gy = ((float)gyro_y_raw / MPU_GYRO_SCALE_500DPS) - gyro_bias_y_dps;
  sample->gz = (((float)gyro_z_raw / MPU_GYRO_SCALE_500DPS) - gyro_bias_z_dps) * GYRO_Z_SIGN;

  rotate_xy(&sample->ax, &sample->ay);
  rotate_xy(&sample->gx, &sample->gy);

  return true;
}

static bool mpu6050_read_filtered(ImuSample *sample, float dt)
{
  ImuSample raw_sample;
  const float gyro_alpha = lpf_alpha(GYRO_LPF_HZ, dt);
  const float accel_alpha = lpf_alpha(ACCEL_LPF_HZ, dt);

  if (!mpu6050_read_raw(&raw_sample)) {
    return false;
  }

  gyro_lpf_x = lpf_apply(gyro_lpf_x, raw_sample.gx, gyro_alpha);
  gyro_lpf_y = lpf_apply(gyro_lpf_y, raw_sample.gy, gyro_alpha);
  gyro_lpf_z = lpf_apply(gyro_lpf_z, raw_sample.gz, gyro_alpha);

  accel_lpf_x = lpf_apply(accel_lpf_x, raw_sample.ax, accel_alpha);
  accel_lpf_y = lpf_apply(accel_lpf_y, raw_sample.ay, accel_alpha);
  accel_lpf_z = lpf_apply(accel_lpf_z, raw_sample.az, accel_alpha);

  sample->ax = accel_lpf_x;
  sample->ay = accel_lpf_y;
  sample->az = accel_lpf_z;
  sample->gx = gyro_lpf_x;
  sample->gy = gyro_lpf_y;
  sample->gz = gyro_lpf_z;

  return true;
}

static bool mpu6050_calibrate(void)
{
  ImuSample sample;
  float gyro_x_sum = 0.0f;
  float gyro_y_sum = 0.0f;
  float gyro_z_sum = 0.0f;
  float accel_x_sum = 0.0f;
  float accel_y_sum = 0.0f;
  float accel_z_sum = 0.0f;

  gyro_bias_x_dps = 0.0f;
  gyro_bias_y_dps = 0.0f;
  gyro_bias_z_dps = 0.0f;

  uart_log("[IMU] Keep frame still and level for calibration...\r\n");

  for (uint32_t i = 0U; i < GYRO_CALIB_SAMPLES; i++) {
    if (!mpu6050_read_raw(&sample)) {
      return false;
    }
    gyro_x_sum += sample.gx;
    gyro_y_sum += sample.gy;
    gyro_z_sum += sample.gz;
    HAL_Delay(2U);
  }

  gyro_bias_x_dps = gyro_x_sum / (float)GYRO_CALIB_SAMPLES;
  gyro_bias_y_dps = gyro_y_sum / (float)GYRO_CALIB_SAMPLES;
  gyro_bias_z_dps = (gyro_z_sum / (float)GYRO_CALIB_SAMPLES) * GYRO_Z_SIGN;

  for (uint32_t i = 0U; i < ACCEL_CALIB_SAMPLES; i++) {
    if (!mpu6050_read_raw(&sample)) {
      return false;
    }
    accel_x_sum += sample.ax;
    accel_y_sum += sample.ay;
    accel_z_sum += sample.az;
    HAL_Delay(2U);
  }

  accel_x_sum /= (float)ACCEL_CALIB_SAMPLES;
  accel_y_sum /= (float)ACCEL_CALIB_SAMPLES;
  accel_z_sum /= (float)ACCEL_CALIB_SAMPLES;

  {
    const float gyro_bias_mag = sqrtf((gyro_bias_x_dps * gyro_bias_x_dps) +
                                      (gyro_bias_y_dps * gyro_bias_y_dps) +
                                      (gyro_bias_z_dps * gyro_bias_z_dps));
    const float accel_mag = sqrtf((accel_x_sum * accel_x_sum) +
                                  (accel_y_sum * accel_y_sum) +
                                  (accel_z_sum * accel_z_sum));

    if ((gyro_bias_mag > 35.0f) || (accel_mag < 0.70f) || (accel_mag > 1.30f)) {
      uart_log("[IMU] Calibration rejected. Keep board still.\r\n");
      return false;
    }
  }

  /* Boot calibration: ONLY calibrate Gyro bias (zero-rate offset).
   * Accel Level Trim is kept at neutral 0.0 deg so arbitrary board slant on boot
   * does NOT become an artificial 0-degree level reference. */
  accel_roll_trim = SENSOR_LEVEL_ROLL_OFFSET_DEG;
  accel_pitch_trim = SENSOR_LEVEL_PITCH_OFFSET_DEG;

  gyro_lpf_x = 0.0f;
  gyro_lpf_y = 0.0f;
  gyro_lpf_z = 0.0f;
  accel_lpf_x = 0.0f;
  accel_lpf_y = 0.0f;
  accel_lpf_z = 1.0f;
  attitude.roll_deg = 0.0f;
  attitude.pitch_deg = 0.0f;
  attitude.yaw_deg = heading_deg;
  accel_norm = 1.0f;
  accel_trust = 1.0f;

  (void)snprintf(uart_buf,
                 sizeof(uart_buf),
                 "[IMU] Calibration done. accel trim=%ld/%ld hover trim=%ld/%ld\r\n",
                 (long)(accel_roll_trim * 10.0f),
                 (long)(accel_pitch_trim * 10.0f),
                 (long)(HOVER_ROLL_TRIM_DEG * 10.0f),
                 (long)(HOVER_PITCH_TRIM_DEG * 10.0f));
  uart_log(uart_buf);
  return true;
}

static void run_level_calibration(void)
{
  ImuSample sample;
  float ax_sum = 0.0f;
  float ay_sum = 0.0f;
  float az_sum = 0.0f;
  const uint16_t samples = 200U;

  uart_log("[CALIB] Starting Level Trim calibration... Keep frame still!\r\n");

  for (uint16_t i = 0U; i < samples; i++) {
    if (mpu6050_read_raw(&sample)) {
      ax_sum += sample.ax;
      ay_sum += sample.ay;
      az_sum += sample.az;
    }
    HAL_Delay(2U);
  }

  sample.ax = ax_sum / (float)samples;
  sample.ay = ay_sum / (float)samples;
  sample.az = az_sum / (float)samples;

  const float accel_mag = sqrtf(sample.ax * sample.ax + sample.ay * sample.ay + sample.az * sample.az);
  if ((accel_mag < 0.70f) || (accel_mag > 1.30f)) {
    uart_log("[CALIB] Level trim rejected: excessive tilt/motion!\r\n");
    return;
  }

  accel_roll_trim = control_roll_accel_deg(&sample);
  accel_pitch_trim = control_pitch_accel_deg(&sample);
  attitude.roll_deg = 0.0f;
  attitude.pitch_deg = 0.0f;

  (void)snprintf(uart_buf,
                 sizeof(uart_buf),
                 "[CALIB] Level Trim SUCCESS! roll_trim=%.1f deg, pitch_trim=%.1f deg\r\n",
                 accel_roll_trim,
                 accel_pitch_trim);
  uart_log(uart_buf);
}

static void update_attitude(const ImuSample *sample, float dt)
{
  const float accel_roll = control_roll_accel_deg(sample) - accel_roll_trim;
  const float accel_pitch = control_pitch_accel_deg(sample) - accel_pitch_trim;
  const float accel_correction_step = clampf(ACCEL_CORRECTION_GAIN * dt, 0.0f, 0.08f);

  accel_norm = sqrtf(sample->ax * sample->ax + sample->ay * sample->ay + sample->az * sample->az);
  accel_trust = 1.0f - clampf(fabsf(accel_norm - 1.0f) / ACCEL_TRUST_MAX_ERROR_G, 0.0f, 1.0f);

  attitude.roll_deg += control_roll_rate_dps(sample) * dt;
  attitude.pitch_deg += control_pitch_rate_dps(sample) * dt;
  attitude.yaw_deg = wrap_heading_deg(attitude.yaw_deg + (control_yaw_rate_dps(sample) * dt));

  if (accel_trust > 0.0f) {
    attitude.roll_deg += (accel_roll - attitude.roll_deg) * accel_correction_step * accel_trust;
    attitude.pitch_deg += (accel_pitch - attitude.pitch_deg) * accel_correction_step * accel_trust;
  }

  heading_deg = attitude.yaw_deg;
}

static bool receiver_read_command(RcCommand *command, uint32_t now_ms)
{
  uint32_t ages[RC_CHANNELS];
  float roll_raw;
  float pitch_raw;
  float yaw_raw;
  float throttle_raw;

  __disable_irq();
  for (uint8_t i = 0U; i < RC_CHANNELS; i++) {
    command->raw[i] = rc_pulse_us[i];
    ages[i] = rc_last_update_ms[i];
  }
  __enable_irq();

  command->valid = true;
  for (uint8_t i = 0U; i < RC_CHANNELS; i++) {
    if ((ages[i] == 0U) ||
        ((uint32_t)(now_ms - ages[i]) > RX_TIMEOUT_MS) ||
        (command->raw[i] < RC_VALID_MIN_US) ||
        (command->raw[i] > RC_VALID_MAX_US)) {
      command->valid = false;
      break;
    }
  }

  if (!command->valid) {
    command->rf_loss_detected = false;
    command->throttle_low = true;
    command->yaw_arm_request = false;
    command->yaw_disarm_request = false;
    command->throttle_norm = 0.0f;
    command->throttle_stick_norm = 0.0f;
    command->roll_cmd = 0.0f;
    command->pitch_cmd = 0.0f;
    command->yaw_cmd = 0.0f;
    return false;
  }

  command->throttle_low = (command->raw[2] <= THROTTLE_LOW_US);
  command->yaw_arm_request = command->throttle_low &&
                              (command->raw[3] >= ARM_YAW_HIGH_US) &&
                              (fabsf((float)command->raw[0] - rc_roll_center_runtime) <= (float)ARM_STICK_CENTER_WINDOW_US) &&
                              (fabsf((float)command->raw[1] - rc_pitch_center_runtime) <= (float)ARM_STICK_CENTER_WINDOW_US);
  command->yaw_disarm_request = command->throttle_low &&
                                 (command->raw[3] <= DISARM_YAW_LOW_US) &&
                                 (fabsf((float)command->raw[0] - rc_roll_center_runtime) <= (float)ARM_STICK_CENTER_WINDOW_US) &&
                                 (fabsf((float)command->raw[1] - rc_pitch_center_runtime) <= (float)ARM_STICK_CENTER_WINDOW_US);

  update_rc_center_learning(command, now_ms);

  roll_raw = normalize_centered(command->raw[0],
                                RC_ROLL_MIN_US,
                                rc_roll_center_runtime,
                                RC_ROLL_MAX_US,
                                RC_ROLL_SIGN);
  pitch_raw = normalize_centered(command->raw[1],
                                 RC_PITCH_MIN_US,
                                 rc_pitch_center_runtime,
                                 RC_PITCH_MAX_US,
                                 RC_PITCH_SIGN);
  throttle_raw = normalize_throttle(command->raw[2]);
  yaw_raw = normalize_centered(command->raw[3],
                               RC_YAW_MIN_US,
                               rc_yaw_center_runtime,
                               RC_YAW_MAX_US,
                               RC_YAW_SIGN);

  rc_roll_filtered = lpf_apply(rc_roll_filtered, roll_raw, RC_STICK_ALPHA);
  rc_pitch_filtered = lpf_apply(rc_pitch_filtered, pitch_raw, RC_STICK_ALPHA);
  rc_yaw_filtered = lpf_apply(rc_yaw_filtered, yaw_raw, RC_YAW_ALPHA);
  rc_throttle_filtered = lpf_apply(rc_throttle_filtered, throttle_raw, RC_THROTTLE_ALPHA);
  command->throttle_stick_norm = rc_throttle_filtered;
  command->throttle_norm = apply_throttle_curve(rc_throttle_filtered);
  command->roll_cmd = rc_roll_filtered;
  command->pitch_cmd = rc_pitch_filtered;
  if (command->throttle_low) {
    rc_yaw_filtered = 0.0f;
    command->yaw_cmd = 0.0f;
  } else {
    command->yaw_cmd = rc_yaw_filtered;
  }
  command->rf_loss_detected = detect_rf_loss_pattern(command);
  if (!command->rf_loss_detected) {
    last_pilot_throttle_norm = command->throttle_norm;
  }

  return true;
}

static void servo_set_us(uint16_t pulse_us)
{
  const uint16_t limited = (uint16_t)clampf((float)pulse_us, 500.0f, 2500.0f);

  servo_output_us = limited;
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, limited);
}

static void servo_grip(void)
{
  servo_released = false;
  servo_set_us(SERVO_GRIP_US);
}

static void servo_open(void)
{
  servo_released = true;
  servo_set_us(SERVO_OPEN_US);
}

static void servo_update_from_ch5(uint32_t now_ms)
{
  uint16_t ch5_us;
  uint32_t last_update_ms;

  __disable_irq();
  ch5_us = servo_ch5_pulse_us;
  last_update_ms = servo_ch5_last_update_ms;
  __enable_irq();

  if ((last_update_ms == 0U) ||
      ((uint32_t)(now_ms - last_update_ms) > SERVO_CH5_TIMEOUT_MS)) {
    servo_grip();
    return;
  }

  if (ch5_us >= SERVO_CH5_OPEN_US) {
    servo_open();
  } else if (ch5_us <= SERVO_CH5_GRIP_US) {
    servo_grip();
  }
}

static bool command_is_centered(const RcCommand *command)
{
  return (fabsf(command->roll_cmd) <= 0.15f) &&
         (fabsf(command->pitch_cmd) <= 0.15f) &&
         (fabsf(command->yaw_cmd) <= 0.20f);
}

static void enter_rf_failsafe(uint32_t now_ms)
{
  failsafe_mode = FAILSAFE_HOLD;
  failsafe_state_ms = now_ms;
  rf_recover_ms = 0U;
  land_detect_ms = 0U;
  failsafe_throttle_norm = clampf(last_pilot_throttle_norm,
                                  FAILSAFE_DESCEND_MIN,
                                  0.65f);
  reset_all_pids();
  uart_log("[SAFE] RF lost. Holding level.\r\n");
}

static void update_failsafe_state(const RcCommand *command,
                                  bool sensors_ok,
                                  const ImuSample *sample,
                                  bool sample_valid,
                                  uint32_t now_ms,
                                  float dt)
{
  bool preset_loss_confirmed = false;
  bool receiver_lost;
  const bool low_motion = sample_valid &&
                          (fabsf(attitude.roll_deg) <= FAILSAFE_LAND_ATTITUDE_DEG) &&
                          (fabsf(attitude.pitch_deg) <= FAILSAFE_LAND_ATTITUDE_DEG) &&
                          (fabsf(control_roll_rate_dps(sample)) <= FAILSAFE_LAND_RATE_DPS) &&
                          (fabsf(control_pitch_rate_dps(sample)) <= FAILSAFE_LAND_RATE_DPS) &&
                          (fabsf(control_yaw_rate_dps(sample)) <= FAILSAFE_LAND_RATE_DPS);

  if (command->rf_loss_detected) {
    if (rf_preset_detect_ms == 0U) {
      rf_preset_detect_ms = now_ms;
    } else if ((uint32_t)(now_ms - rf_preset_detect_ms) >= RF_PRESET_DETECT_MS) {
      preset_loss_confirmed = true;
    }
  } else {
    rf_preset_detect_ms = 0U;
  }

  receiver_lost = (!command->valid) || preset_loss_confirmed;

  if (!sensors_ok) {
    failsafe_active = true;
    failsafe_mode = FAILSAFE_NONE;
    if (armed) {
      uart_log("[SAFE] IMU unavailable. Emergency disarm.\r\n");
    }
    armed = false;
    reset_all_pids();
    return;
  }

  if (!armed) {
    failsafe_active = receiver_lost;
    failsafe_mode = FAILSAFE_NONE;
    rf_recover_ms = 0U;
    rf_preset_detect_ms = 0U;
    land_detect_ms = 0U;
    return;
  }

  if (receiver_lost) {
    failsafe_active = true;
    rf_recover_ms = 0U;

    if (failsafe_mode == FAILSAFE_NONE || failsafe_mode == FAILSAFE_RECOVER) {
      if (last_pilot_throttle_norm > FAILSAFE_AIRBORNE_THROTTLE) {
        enter_rf_failsafe(now_ms);
      } else {
        uart_log("[SAFE] RF lost on ground. Disarming.\r\n");
        armed = false;
        failsafe_mode = FAILSAFE_NONE;
        rf_preset_detect_ms = 0U;
        reset_all_pids();
      }
      return;
    }

    if ((failsafe_mode == FAILSAFE_HOLD) &&
        ((uint32_t)(now_ms - failsafe_state_ms) >= FAILSAFE_HOLD_MS)) {
      failsafe_mode = FAILSAFE_DESCEND;
      failsafe_state_ms = now_ms;
      uart_log("[SAFE] Controlled descent active.\r\n");
    }

    if (failsafe_mode == FAILSAFE_DESCEND) {
      const float descend_rate = (failsafe_throttle_norm > FAILSAFE_DESCEND_SPLIT) ?
                                 FAILSAFE_DESCEND_RATE_HIGH :
                                 FAILSAFE_DESCEND_RATE_LOW;

      failsafe_throttle_norm = fmaxf(FAILSAFE_DESCEND_FINAL,
                                     failsafe_throttle_norm - (descend_rate * dt));

      if ((failsafe_throttle_norm <= (FAILSAFE_DESCEND_FINAL + 0.01f)) && low_motion) {
        if (land_detect_ms == 0U) {
          land_detect_ms = now_ms;
        } else if ((uint32_t)(now_ms - land_detect_ms) >= FAILSAFE_LAND_STABLE_MS) {
          uart_log("[SAFE] Landing detected. Disarming.\r\n");
          armed = false;
          failsafe_mode = FAILSAFE_NONE;
          failsafe_active = false;
          reset_all_pids();
          stop_all_motors();
        }
      } else {
        land_detect_ms = 0U;
      }
    }

    return;
  }

  if ((failsafe_mode == FAILSAFE_HOLD) || (failsafe_mode == FAILSAFE_DESCEND)) {
    failsafe_active = true;
    if (rf_recover_ms == 0U) {
      rf_recover_ms = now_ms;
    } else if (((uint32_t)(now_ms - rf_recover_ms) >= FAILSAFE_RECOVER_VALID_MS) &&
               command_is_centered(command)) {
      failsafe_mode = FAILSAFE_RECOVER;
      failsafe_state_ms = now_ms;
      uart_log("[SAFE] RF restored. Blending pilot control.\r\n");
    }
    return;
  }

  if (failsafe_mode == FAILSAFE_RECOVER) {
    failsafe_active = true;
    if ((uint32_t)(now_ms - failsafe_state_ms) >= FAILSAFE_RECOVER_BLEND_MS) {
      failsafe_mode = FAILSAFE_NONE;
      failsafe_active = false;
      rf_recover_ms = 0U;
      rf_preset_detect_ms = 0U;
      uart_log("[SAFE] Pilot control restored.\r\n");
    }
    return;
  }

  failsafe_active = false;
}

static void update_stick_commands(const RcCommand *command, uint32_t now_ms)
{
  static uint32_t level_calib_req_ms = 0U;
  static uint32_t mag_calib_req_ms = 0U;

  if (armed || !command->valid || !command->throttle_low ||
      command->yaw_arm_request || command->yaw_disarm_request) {
    level_calib_req_ms = 0U;
    mag_calib_req_ms = 0U;
    return;
  }

  const bool yaw_left = (command->raw[3] <= DISARM_YAW_LOW_US);
  const bool roll_centered = (fabsf((float)command->raw[0] - rc_roll_center_runtime) <= (float)ARM_STICK_CENTER_WINDOW_US);
  const bool pitch_up = (command->raw[1] >= (RC_PITCH_MAX_US - 100U));
  const bool pitch_down = (command->raw[1] <= (RC_PITCH_MIN_US + 100U));

  /* 1. Stick Command: Level Trim (Ga MIN + Yaw TRAI + Pitch LEN) */
  if (yaw_left && roll_centered && pitch_up) {
    if (level_calib_req_ms == 0U) {
      level_calib_req_ms = now_ms;
    } else if ((uint32_t)(now_ms - level_calib_req_ms) >= 2500U) {
      level_calib_req_ms = 0U;
      run_level_calibration();
    }
  } else {
    level_calib_req_ms = 0U;
  }

  /* 2. Stick Command: Mag 360 Calib (Ga MIN + Yaw TRAI + Pitch XUONG) */
  if (yaw_left && roll_centered && pitch_down) {
    if (mag_calib_req_ms == 0U) {
      mag_calib_req_ms = now_ms;
    } else if ((uint32_t)(now_ms - mag_calib_req_ms) >= 2500U) {
      mag_calib_req_ms = 0U;
      if (!mag_calib_active) {
        mag_calib_active = true;
        mag_calib_start_ms = now_ms;
        mag_calib_min_x = 999.0f; mag_calib_max_x = -999.0f;
        mag_calib_min_y = 999.0f; mag_calib_max_y = -999.0f;
        mag_calib_min_z = 999.0f; mag_calib_max_z = -999.0f;
        uart_log("[MAG] 360-deg Calibration STARTED! Rotate drone in all 3 axes for 20s...\r\n");
      }
    }
  } else {
    mag_calib_req_ms = 0U;
  }
}

static void update_arming_state(const RcCommand *command, bool sensors_ok, uint32_t now_ms)
{
  if (!sensors_ok) {
    armed = false;
    arm_request_ms = 0U;
    disarm_request_ms = 0U;
    reset_all_pids();
    return;
  }

  if (!command->valid || (failsafe_mode != FAILSAFE_NONE)) {
    arm_request_ms = 0U;
    disarm_request_ms = 0U;
    return;
  }

  if (command->yaw_disarm_request) {
    if (disarm_request_ms == 0U) {
      disarm_request_ms = now_ms;
    } else if ((uint32_t)(now_ms - disarm_request_ms) >= DISARM_HOLD_MS) {
      if (armed) {
        uart_log("[ARM] Disarmed by stick command.\r\n");
      }
      armed = false;
      heading_hold_engaged = false;
      rc_yaw_filtered = 0.0f;
      arm_request_ms = 0U;
      reset_all_pids();
    }
  } else {
    disarm_request_ms = 0U;
  }

  if (armed) {
    return;
  }

  /* Kiem tra cac lenh Stick khi chua ARM */
  update_stick_commands(command, now_ms);

  if (!command->yaw_arm_request) {
    arm_request_ms = 0U;
    return;
  }

  if (arm_request_ms == 0U) {
    arm_request_ms = now_ms;
  } else if ((uint32_t)(now_ms - arm_request_ms) >= ARM_HOLD_MS) {
    armed = true;
    disarm_request_ms = 0U;
    rc_yaw_filtered = 0.0f;
    heading_hold_ref_deg = attitude.yaw_deg;
    heading_hold_engaged = false;
    last_heading_hold_ms = now_ms;
    reset_all_pids();

    /* Thiet lap moc 0.0m tuc thi cho Baro va chot Home cho GPS */
    if (bmp388_is_ready()) {
      bmp388_zero_altitude();
    }
    gps_set_home();

    uart_log("[ARM] Armed. Baro zeroed & GPS home set.\r\n");
  }
}

static void build_control_command(const RcCommand *pilot_command,
                                  RcCommand *control_command,
                                  uint32_t now_ms,
                                  float dt)
{
  float target_throttle_norm;

  *control_command = *pilot_command;

  target_throttle_norm = pilot_command->throttle_norm;

  if ((failsafe_mode == FAILSAFE_NONE) || !armed) {
    control_command->throttle_norm = apply_throttle_slew(target_throttle_norm, dt, !armed);
    return;
  }

  control_command->valid = true;
  control_command->yaw_arm_request = false;
  control_command->yaw_disarm_request = false;

  if ((failsafe_mode == FAILSAFE_HOLD) || (failsafe_mode == FAILSAFE_DESCEND)) {
    control_command->roll_cmd = 0.0f;
    control_command->pitch_cmd = 0.0f;
    control_command->yaw_cmd = 0.0f;
    target_throttle_norm = failsafe_throttle_norm;
  } else if (failsafe_mode == FAILSAFE_RECOVER) {
    const float blend = clampf((float)((uint32_t)(now_ms - failsafe_state_ms)) /
                               (float)FAILSAFE_RECOVER_BLEND_MS,
                               0.0f,
                               1.0f);

    control_command->roll_cmd = pilot_command->roll_cmd * blend;
    control_command->pitch_cmd = pilot_command->pitch_cmd * blend;
    control_command->yaw_cmd = pilot_command->yaw_cmd * blend;
    target_throttle_norm = failsafe_throttle_norm +
                           ((pilot_command->throttle_norm - failsafe_throttle_norm) * blend);
  }

  control_command->throttle_norm = apply_throttle_slew(target_throttle_norm, dt, false);

  control_command->throttle_low = (control_command->throttle_norm <= 0.03f);

  if (failsafe_mode == FAILSAFE_DESCEND) {
    failsafe_throttle_norm = control_command->throttle_norm;
  }

  (void)dt;
}

static void motor_set_us(uint8_t motor_index, uint16_t pulse_us)
{
  const uint16_t limited = (uint16_t)clampf((float)pulse_us, (float)MOTOR_STOP_US, (float)MOTOR_MAX_US);
  motor_output_us[motor_index] = limited;

  switch (motor_index) {
    case 0U:
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, limited);
      break;
    case 1U:
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, limited);
      break;
    case 2U:
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, limited);
      break;
    case 3U:
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, limited);
      break;
    default:
      break;
  }
}

static void motors_write_all(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4)
{
  motor_set_us(0U, m1);
  motor_set_us(1U, m2);
  motor_set_us(2U, m3);
  motor_set_us(3U, m4);
}

static void stop_all_motors(void)
{
  motors_write_all(MOTOR_STOP_US, MOTOR_STOP_US, MOTOR_STOP_US, MOTOR_STOP_US);
}

static void mix_motor_outputs(float throttle_base,
                              float roll_output,
                              float pitch_output,
                              float yaw_output,
                              float left_cg_bias_us)
{
  float corrections[4];
  float rp_corrections[4];
  float yaw_corrections[4];
  float positive_demand = 0.0f;
  float negative_demand = 0.0f;
  float scale = 1.0f;
  const float yaw_mix_output = yaw_output * YAW_MIX_SIGN;
  const float upper_room = (float)MOTOR_MAX_US - throttle_base;
  const float lower_room = throttle_base - (float)MOTOR_IDLE_US;

  rp_corrections[0] = -roll_output - pitch_output + left_cg_bias_us; // M1 front-left
  rp_corrections[1] = -roll_output + pitch_output + left_cg_bias_us; // M2 rear-left
  rp_corrections[2] =  roll_output + pitch_output - left_cg_bias_us; // M3 rear-right
  rp_corrections[3] =  roll_output - pitch_output - left_cg_bias_us; // M4 front-right
  yaw_corrections[0] = yaw_mix_output;
  yaw_corrections[1] = -yaw_mix_output;
  yaw_corrections[2] = yaw_mix_output;
  yaw_corrections[3] = -yaw_mix_output;

  mixer_yaw_scale = 1.0f;
  mixer_rp_scale = 1.0f;
  mixer_saturated = false;

  for (uint8_t i = 0U; i < 4U; i++) {
    corrections[i] = rp_corrections[i] + yaw_corrections[i];
  }

  for (uint8_t i = 0U; i < 4U; i++) {
    if (corrections[i] > positive_demand) {
      positive_demand = corrections[i];
    }
    if (-corrections[i] > negative_demand) {
      negative_demand = -corrections[i];
    }
  }

  if ((positive_demand > upper_room) && (positive_demand > 0.0f)) {
    scale = fminf(scale, upper_room / positive_demand);
  }

  if ((negative_demand > lower_room) && (negative_demand > 0.0f)) {
    scale = fminf(scale, lower_room / negative_demand);
  }

  if (scale < 0.999f) {
    float low = 0.0f;
    float high = 1.0f;
    mixer_saturated = true;

    for (uint8_t step = 0U; step < 8U; step++) {
      const float mid = (low + high) * 0.5f;
      float pos = 0.0f;
      float neg = 0.0f;

      for (uint8_t i = 0U; i < 4U; i++) {
        const float value = rp_corrections[i] + (yaw_corrections[i] * mid);
        if (value > pos) {
          pos = value;
        }
        if (-value > neg) {
          neg = -value;
        }
      }

      if ((pos <= upper_room) && (neg <= lower_room)) {
        low = mid;
      } else {
        high = mid;
      }
    }

    mixer_yaw_scale = (fabsf(yaw_output) > 1.0f) ? fmaxf(low, YAW_AUTHORITY_FLOOR) : low;
    low = 0.0f;
    high = 1.0f;
    for (uint8_t step = 0U; step < 8U; step++) {
      const float mid = (low + high) * 0.5f;
      float pos = 0.0f;
      float neg = 0.0f;

      for (uint8_t i = 0U; i < 4U; i++) {
        const float value = (rp_corrections[i] * mid) + (yaw_corrections[i] * mixer_yaw_scale);
        if (value > pos) {
          pos = value;
        }
        if (-value > neg) {
          neg = -value;
        }
      }

      if ((pos <= upper_room) && (neg <= lower_room)) {
        low = mid;
      } else {
        high = mid;
      }
    }

    mixer_rp_scale = low;

    for (uint8_t i = 0U; i < 4U; i++) {
      corrections[i] = (rp_corrections[i] * mixer_rp_scale) +
                       (yaw_corrections[i] * mixer_yaw_scale);
    }
    scale = 1.0f;
  }

  motors_write_all((uint16_t)clampf(throttle_base + corrections[0] * scale,
                                    (float)MOTOR_IDLE_US,
                                    (float)MOTOR_MAX_US),
                   (uint16_t)clampf(throttle_base + corrections[1] * scale,
                                    (float)MOTOR_IDLE_US,
                                    (float)MOTOR_MAX_US),
                   (uint16_t)clampf(throttle_base + corrections[2] * scale,
                                    (float)MOTOR_IDLE_US,
                                    (float)MOTOR_MAX_US),
                   (uint16_t)clampf(throttle_base + corrections[3] * scale,
                                    (float)MOTOR_IDLE_US,
                                    (float)MOTOR_MAX_US));
}

static void run_flight_control(const RcCommand *command, const ImuSample *sample, float dt)
{
  float roll_outer_kp;
  float pitch_outer_kp;
  float desired_roll_angle;
  float desired_pitch_angle;
  float desired_roll_rate;
  float desired_pitch_rate;
  float desired_yaw_rate;
  float roll_output;
  float pitch_output;
  float yaw_output;
  float throttle_base;
  float left_cg_takeoff_bias_us;
  const bool enable_integrator = (command->throttle_norm > 0.10f);
  const float trim_scale = hover_trim_scale(command->throttle_norm);
  const bool roll_pitch_stick_active = (fabsf(command->roll_cmd) > HEADING_HOLD_MAX_RP_STICK) ||
                                       (fabsf(command->pitch_cmd) > HEADING_HOLD_MAX_RP_STICK);
  const bool roll_pitch_moving = (fabsf(control_roll_rate_dps(sample)) > HEADING_HOLD_MAX_RP_RATE_DPS) ||
                                 (fabsf(control_pitch_rate_dps(sample)) > HEADING_HOLD_MAX_RP_RATE_DPS);
  const bool roll_pitch_tilted = (fabsf(attitude.roll_deg) > HEADING_HOLD_MAX_TILT_DEG) ||
                                 (fabsf(attitude.pitch_deg) > HEADING_HOLD_MAX_TILT_DEG);
  const bool roll_pitch_maneuver = roll_pitch_stick_active || roll_pitch_moving || roll_pitch_tilted;
  const bool sticks_centered_for_takeoff = (fabsf(command->roll_cmd) <= TAKEOFF_ASSIST_STICK_THRESH) &&
                                           (fabsf(command->pitch_cmd) <= TAKEOFF_ASSIST_STICK_THRESH) &&
                                           (fabsf(command->yaw_cmd) <= HEADING_HOLD_YAW_DEADBAND);
  const bool yaw_control_active = (!command->throttle_low) &&
                                  (command->throttle_norm >= YAW_CONTROL_MIN_THROTTLE);

  if (!armed || (failsafe_active && (failsafe_mode == FAILSAFE_NONE))) {
    throttle_delta_norm = 0.0f;
    last_control_throttle_norm = 0.0f;
    yaw_dynamic_damping_output = 0.0f;
    yaw_throttle_feedforward_output = 0.0f;
    yaw_feedforward_output = 0.0f;
    yaw_maneuver_bias_output = 0.0f;
    yaw_bias_output *= YAW_BIAS_DECAY;
    heading_hold_limited_error_deg = 0.0f;
    throttle_rate_norm_s = 0.0f;
    takeoff_assist_scale = 0.0f;
    roll_pitch_dynamic = false;
    stop_all_motors();
    return;
  }

  throttle_delta_norm = command->throttle_norm - last_control_throttle_norm;
  throttle_rate_norm_s = throttle_delta_norm / fmaxf(dt, 0.001f);
  last_control_throttle_norm = command->throttle_norm;
  roll_pitch_dynamic = roll_pitch_maneuver;

  takeoff_assist_scale = 0.0f;
  if (sticks_centered_for_takeoff &&
      (command->throttle_norm >= TAKEOFF_ASSIST_MIN_THROTTLE) &&
      (command->throttle_norm <= TAKEOFF_ASSIST_MAX_THROTTLE)) {
    takeoff_assist_scale = clampf((TAKEOFF_ASSIST_MAX_THROTTLE - command->throttle_norm) /
                                  (TAKEOFF_ASSIST_MAX_THROTTLE - TAKEOFF_ASSIST_MIN_THROTTLE),
                                  0.0f,
                                  1.0f);
  }
  left_cg_takeoff_bias_us = TAKEOFF_LEFT_CG_BIAS_US * takeoff_assist_scale;

  desired_roll_angle = (command->roll_cmd * MAX_ANGLE_DEG) +
                       (HOVER_ROLL_TRIM_DEG * trim_scale) +
                       (TAKEOFF_ROLL_TRIM_DEG * takeoff_assist_scale);
  desired_pitch_angle = (command->pitch_cmd * MAX_ANGLE_DEG) +
                        (HOVER_PITCH_TRIM_DEG * trim_scale) +
                        (TAKEOFF_PITCH_TRIM_DEG * takeoff_assist_scale);

  roll_outer_kp = ANGLE_OUTER_KP +
                  ((fabsf(command->roll_cmd) <= CENTER_BRAKE_STICK_THRESH) ? CENTER_BRAKE_KP_BOOST : 0.0f) +
                  (TAKEOFF_ASSIST_KP_BOOST * takeoff_assist_scale);
  pitch_outer_kp = ANGLE_OUTER_KP +
                   ((fabsf(command->pitch_cmd) <= CENTER_BRAKE_STICK_THRESH) ? CENTER_BRAKE_KP_BOOST : 0.0f) +
                   (TAKEOFF_ASSIST_KP_BOOST * takeoff_assist_scale);

  desired_roll_rate = clampf((desired_roll_angle - attitude.roll_deg) * roll_outer_kp,
                             -MAX_ANGLE_RATE_DPS,
                             MAX_ANGLE_RATE_DPS);
  desired_pitch_rate = clampf((desired_pitch_angle - attitude.pitch_deg) * pitch_outer_kp,
                              -MAX_ANGLE_RATE_DPS,
                              MAX_ANGLE_RATE_DPS);
  heading_hold_allowed = yaw_control_active && (command->throttle_norm >= HEADING_HOLD_MIN_THROTTLE);
  yaw_maneuver_scale = 1.0f;

  if (yaw_control_active && (fabsf(command->yaw_cmd) > HEADING_HOLD_YAW_EXIT_DB)) {
    desired_yaw_rate = command->yaw_cmd * MAX_YAW_RATE_DPS;
    heading_hold_ref_deg = attitude.yaw_deg;
    heading_hold_engaged = false;
    last_heading_hold_ms = HAL_GetTick();
  } else if (heading_hold_allowed) {
    const uint32_t now_ms = HAL_GetTick();

    if (!heading_hold_engaged &&
        (fabsf(command->yaw_cmd) <= HEADING_HOLD_YAW_DEADBAND) &&
        ((uint32_t)(now_ms - last_heading_hold_ms) >= HEADING_HOLD_RELEASE_MS)) {
      heading_hold_ref_deg = attitude.yaw_deg;
      heading_hold_engaged = true;
    }

    if (heading_hold_engaged) {
      heading_hold_limited_error_deg = clampf(heading_error_deg(heading_hold_ref_deg, attitude.yaw_deg),
                                             -HEADING_HOLD_ERROR_LIMIT_DEG,
                                             HEADING_HOLD_ERROR_LIMIT_DEG);
      desired_yaw_rate = clampf(heading_hold_limited_error_deg * HEADING_HOLD_KP,
                                -HEADING_HOLD_RATE_LIMIT_DPS,
                                HEADING_HOLD_RATE_LIMIT_DPS);
    } else {
      heading_hold_limited_error_deg = 0.0f;
      desired_yaw_rate = 0.0f;
    }
  } else {
    heading_hold_ref_deg = attitude.yaw_deg;
    heading_hold_engaged = false;
    last_heading_hold_ms = HAL_GetTick();
    desired_yaw_rate = 0.0f;
    heading_hold_limited_error_deg = 0.0f;
  }

  if (roll_pitch_maneuver && (fabsf(command->yaw_cmd) <= HEADING_HOLD_YAW_DEADBAND)) {
    const float stick_amount = fmaxf(fabsf(command->roll_cmd), fabsf(command->pitch_cmd));
    const float tilt_amount = fmaxf(fabsf(attitude.roll_deg), fabsf(attitude.pitch_deg)) / MAX_ANGLE_DEG;
    const float rate_amount = fmaxf(fabsf(control_roll_rate_dps(sample)),
                                    fabsf(control_pitch_rate_dps(sample))) / MAX_ANGLE_RATE_DPS;
    const float maneuver_amount = clampf(fmaxf(stick_amount, fmaxf(tilt_amount, rate_amount)),
                                         0.0f,
                                         1.0f);
    yaw_maneuver_scale = 1.0f - clampf((maneuver_amount - YAW_MANEUEVER_STICK_THRESH) /
                                       (1.0f - YAW_MANEUEVER_STICK_THRESH),
                                       0.0f,
                                       1.0f) * (1.0f - YAW_MANEUVER_AUTHORITY_MIN);
  }

  last_desired_yaw_rate = desired_yaw_rate;
  last_gyro_x_dps = control_pitch_rate_dps(sample);
  last_gyro_y_dps = control_roll_rate_dps(sample);
  last_heading_error_deg = heading_hold_engaged ? heading_hold_limited_error_deg : 0.0f;
  last_gyro_z_dps = control_yaw_rate_dps(sample);

  if (mixer_saturated && (mixer_yaw_scale < 0.85f) && heading_hold_engaged) {
    pid_yaw.integrator *= YAW_SAT_INTEGRATOR_DECAY;
  }

  roll_output = pid_update(&pid_roll, desired_roll_rate, control_roll_rate_dps(sample), dt, enable_integrator);
  pitch_output = pid_update(&pid_pitch, desired_pitch_rate, control_pitch_rate_dps(sample), dt, enable_integrator);
  if (yaw_control_active) {
    yaw_output = pid_update(&pid_yaw,
                            desired_yaw_rate,
                            control_yaw_rate_dps(sample),
                            dt,
                            enable_integrator);
  } else {
    pid_reset(&pid_yaw);
    yaw_output = 0.0f;
  }

  yaw_dynamic_damping_output = 0.0f;
  yaw_throttle_feedforward_output = 0.0f;
  yaw_feedforward_output = 0.0f;
  yaw_maneuver_bias_output = 0.0f;
  if (yaw_control_active && (fabsf(command->yaw_cmd) <= HEADING_HOLD_YAW_DEADBAND)) {
    if (heading_hold_allowed || roll_pitch_maneuver ||
        (fabsf(throttle_delta_norm) > MAG_CORRECTION_MAX_THR_DELTA)) {
      const float yaw_damping_kp = YAW_DYNAMIC_DAMPING_KP +
                                   (command->throttle_norm * YAW_DYNAMIC_DAMPING_THR_KP) +
                                   (roll_pitch_maneuver ? YAW_DYNAMIC_DAMPING_RP_KP : 0.0f);
      yaw_dynamic_damping_output = -control_yaw_rate_dps(sample) * yaw_damping_kp;
    }
    if (roll_pitch_maneuver) {
      const float rp_stick_amount = fmaxf(fabsf(command->roll_cmd), fabsf(command->pitch_cmd));
      const float rp_output_amount = fmaxf(fabsf(roll_output), fabsf(pitch_output));
      const float rp_rate_amount = fmaxf(fabsf(control_roll_rate_dps(sample)),
                                        fabsf(control_pitch_rate_dps(sample)));
      const float rp_throttle_scale = 1.0f + (command->throttle_norm * YAW_RP_BIAS_THR_GAIN);
      const float rp_bias_raw = (rp_stick_amount * YAW_RP_MANEUVER_BIAS_FF) +
                                (rp_output_amount * YAW_RP_OUTPUT_BIAS_FF) +
                                (rp_rate_amount * YAW_RP_RATE_BIAS_FF);
      yaw_maneuver_bias_output = clampf(rp_bias_raw * rp_throttle_scale,
                                        -YAW_RP_BIAS_LIMIT,
                                        YAW_RP_BIAS_LIMIT);
    }
    yaw_throttle_feedforward_output = (command->throttle_norm * YAW_THROTTLE_FF) +
                                      (command->throttle_norm * command->throttle_norm * YAW_THROTTLE_HIGH_FF) +
                                      (throttle_rate_norm_s * YAW_THROTTLE_DELTA_FF) +
                                      (takeoff_assist_scale * YAW_TAKEOFF_BIAS_FF);
    yaw_feedforward_output = yaw_throttle_feedforward_output +
                             (command->roll_cmd * YAW_ROLL_COUPLING_FF) +
                             (command->pitch_cmd * YAW_PITCH_COUPLING_FF) +
                             yaw_maneuver_bias_output;
    if (heading_hold_engaged &&
        (fabsf(desired_yaw_rate) <= YAW_BIAS_LEARN_MAX_CMD_DPS) &&
        (fabsf(heading_hold_limited_error_deg) <= YAW_BIAS_LEARN_MAX_ERROR_DEG) &&
        (fabsf(control_yaw_rate_dps(sample)) <= YAW_BIAS_LEARN_MAX_RATE_DPS) &&
        (command->throttle_norm >= YAW_BIAS_LEARN_MIN_THROTTLE) &&
        (mixer_yaw_scale > 0.80f) &&
        !mixer_saturated) {
      yaw_bias_output += (desired_yaw_rate - control_yaw_rate_dps(sample)) *
                         YAW_BIAS_LEARN_GAIN * dt;
      yaw_bias_output = clampf(yaw_bias_output, -YAW_BIAS_LIMIT, YAW_BIAS_LIMIT);
    } else {
      yaw_bias_output *= mixer_saturated ? YAW_BIAS_SAT_DECAY : YAW_BIAS_DECAY;
    }
    yaw_output += yaw_dynamic_damping_output + yaw_feedforward_output + yaw_bias_output;
    yaw_output = clampf(yaw_output, -pid_yaw.output_limit, pid_yaw.output_limit);
  } else {
    yaw_bias_output *= YAW_BIAS_DECAY;
  }
  last_roll_pid_output = roll_output;
  last_pitch_pid_output = pitch_output;
  last_yaw_pid_output = yaw_output;

  throttle_base = (float)MOTOR_IDLE_US +
                  (command->throttle_norm * (float)((MOTOR_MAX_US - MOTOR_HEADROOM_US) - MOTOR_IDLE_US));
  throttle_tilt_boost = 0.0f;
  if (command->throttle_norm >= TILT_THROTTLE_MIN) {
    const float roll_cos = cosf(attitude.roll_deg * DEG_TO_RAD);
    const float pitch_cos = cosf(attitude.pitch_deg * DEG_TO_RAD);
    const float vertical_factor = clampf(roll_cos * pitch_cos, 0.70f, 1.0f);
    throttle_tilt_boost = clampf((1.0f / vertical_factor) - 1.0f,
                                 0.0f,
                                 TILT_THROTTLE_MAX_BOOST);
    throttle_base += throttle_tilt_boost * command->throttle_norm *
                     (float)((MOTOR_MAX_US - MOTOR_HEADROOM_US) - MOTOR_IDLE_US);
    throttle_base = clampf(throttle_base, (float)MOTOR_IDLE_US, (float)(MOTOR_MAX_US - MOTOR_HEADROOM_US));
  }

  mix_motor_outputs(throttle_base,
                    roll_output,
                    pitch_output,
                    yaw_output,
                    left_cg_takeoff_bias_us);
}

static void FLASH_OPT log_status(const RcCommand *command)
{
  const long roll10 = (long)(attitude.roll_deg * 10.0f);
  const long pitch10 = (long)(attitude.pitch_deg * 10.0f);
  const long roll_cmd100 = (long)(command->roll_cmd * 100.0f);
  const long pitch_cmd100 = (long)(command->pitch_cmd * 100.0f);
  const long yaw_cmd100 = (long)(command->yaw_cmd * 100.0f);
  const long throttle_cmd100 = (long)(command->throttle_norm * 100.0f);
  const long failsafe_throttle100 = (long)(failsafe_throttle_norm * 100.0f);
  const long heading10 = (long)(heading_deg * 10.0f);
  const long heading_ref10 = (long)(heading_hold_ref_deg * 10.0f);
  const long mag_heading10 = (long)(mag_heading_deg * 10.0f);
  const long magx100 = (long)(mag_raw_x * 100.0f);
  const long magy100 = (long)(mag_raw_y * 100.0f);
  const long magz100 = (long)(mag_raw_z * 100.0f);
  const long accel_norm100 = (long)(accel_norm * 100.0f);
  const long mag_field100 = (long)(mag_field_strength * 100.0f);
  const long tilt_boost100 = (long)(throttle_tilt_boost * 100.0f);
  const long takeoff_assist100 = (long)(takeoff_assist_scale * 100.0f);
  const long pid_roll10 = (long)(last_roll_pid_output * 10.0f);
  const long pid_pitch10 = (long)(last_pitch_pid_output * 10.0f);
  const long pid_yaw10 = (long)(last_yaw_pid_output * 10.0f);
  const long desired_yaw10 = (long)(last_desired_yaw_rate * 10.0f);
  const long gyro_x10 = (long)(last_gyro_x_dps * 10.0f);
  const long gyro_y10 = (long)(last_gyro_y_dps * 10.0f);
  const long gyro_z10 = (long)(last_gyro_z_dps * 10.0f);
  const long heading_error10 = (long)(last_heading_error_deg * 10.0f);
  const long throttle_delta1000 = (long)(throttle_delta_norm * 1000.0f);
  const long throttle_rate100 = (long)(throttle_rate_norm_s * 100.0f);
  const long yaw_dynamic10 = (long)(yaw_dynamic_damping_output * 10.0f);
  const long yaw_throttle10 = (long)(yaw_throttle_feedforward_output * 10.0f);
  const long yaw_feedforward10 = (long)(yaw_feedforward_output * 10.0f);
  const long yaw_bias10 = (long)(yaw_maneuver_bias_output * 10.0f);
  const long yaw_learn_bias10 = (long)(yaw_bias_output * 10.0f);
  const long yaw_scale100 = (long)(mixer_yaw_scale * 100.0f);
  const long rp_scale100 = (long)(mixer_rp_scale * 100.0f);
  const long yaw_maneuver100 = (long)(yaw_maneuver_scale * 100.0f);
  const uint16_t servo_ch5_us = servo_ch5_pulse_us;
  const uint16_t servo_us = servo_output_us;

  (void)snprintf(uart_buf,
                 sizeof(uart_buf),
                    "ARM=%u FS=%u FM=%u RL=%u TL=%u AR=%u DR=%u RAW=%u/%u/%u/%u S5=%u SV=%u SR=%u CMD=%ld/%ld/%ld/%ld FTH=%ld CTR=%ld/%ld/%ld THR=%u-%u ANG=%ld/%ld HDG=%ld REF=%ld HH=%u HA=%u MAG=%u MU=%u MA=%u MC=%u MH=%ld MF=%ld ACC=%ld TB=%ld TA=%ld TD=%ld TR=%ld DYR=%ld GX=%ld GY=%ld GZ=%ld HE=%ld YD=%ld YT=%ld YF=%ld YB=%ld YBL=%ld PID=%ld/%ld/%ld SAT=%u YS=%ld RS=%ld YM=%ld MX=%ld/%ld/%ld M=%u/%u/%u/%u\r\n",
                 armed ? 1U : 0U,
                 failsafe_active ? 1U : 0U,
                 (unsigned int)failsafe_mode,
                 command->rf_loss_detected ? 1U : 0U,
                 command->throttle_low ? 1U : 0U,
                 command->yaw_arm_request ? 1U : 0U,
                 command->yaw_disarm_request ? 1U : 0U,
                 (unsigned int)command->raw[0],
                  (unsigned int)command->raw[1],
                  (unsigned int)command->raw[2],
                  (unsigned int)command->raw[3],
                  (unsigned int)servo_ch5_us,
                  (unsigned int)servo_us,
                   servo_released ? 1U : 0U,
                  roll_cmd100,
                  pitch_cmd100,
                  yaw_cmd100,
                  throttle_cmd100,
                  failsafe_throttle100,
                  (long)rc_roll_center_runtime,
                  (long)rc_pitch_center_runtime,
                  (long)rc_yaw_center_runtime,
                  (unsigned int)RC_THROTTLE_MIN_US,
                  (unsigned int)RC_THROTTLE_MAX_US,
                  roll10,
                  pitch10,
                  heading10,
                  heading_ref10,
                  heading_hold_engaged ? 1U : 0U,
                  heading_hold_allowed ? 1U : 0U,
                  mag_ready ? 1U : 0U,
                  mag_usable ? 1U : 0U,
                  mag_correction_allowed ? 1U : 0U,
                  mag_calibrated ? 1U : 0U,
                  mag_heading10,
                  mag_field100,
                  accel_norm100,
                  tilt_boost100,
                  takeoff_assist100,
                  throttle_delta1000,
                  throttle_rate100,
                   desired_yaw10,
                   gyro_x10,
                   gyro_y10,
                   gyro_z10,
                   heading_error10,
                   yaw_dynamic10,
                   yaw_throttle10,
                   yaw_feedforward10,
                    yaw_bias10,
                    yaw_learn_bias10,
                  pid_roll10,
                  pid_pitch10,
                  pid_yaw10,
                  mixer_saturated ? 1U : 0U,
                  yaw_scale100,
                  rp_scale100,
                  yaw_maneuver100,
                  magx100,
                  magy100,
                 magz100,
                 (unsigned int)motor_output_us[0],
                 (unsigned int)motor_output_us[1],
                 (unsigned int)motor_output_us[2],
                 (unsigned int)motor_output_us[3]);

  uart_log(uart_buf);
}

/* Telemetry: chup nhanh trang thai bay cho module nRF24 (chi DOC, khong sua gi).
 * Duoc goi tu telemetry_poll() trong main context -> khong can khoa. */
static void telemetry_fill_cb(TelemetryData *out, const void *ctx)
{
  const RcCommand *command = (const RcCommand *)ctx;
  uint16_t flags = 0U;

  if (armed)                 { flags |= TEL_FLAG_ARMED; }
  if (failsafe_active)       { flags |= TEL_FLAG_FAILSAFE; }
  if (imu_ready)             { flags |= TEL_FLAG_IMU_READY; }
  if (mag_ready)             { flags |= TEL_FLAG_MAG_READY; }
  if (mag_usable)            { flags |= TEL_FLAG_MAG_USABLE; }
  if (mag_correction_allowed){ flags |= TEL_FLAG_MAG_CORR_ALLOWED; }
  if (mag_calibrated)        { flags |= TEL_FLAG_MAG_CALIBRATED; }
  if (heading_hold_engaged)  { flags |= TEL_FLAG_HEADING_HOLD; }
  if (heading_hold_allowed)  { flags |= TEL_FLAG_HEADING_HOLD_OK; }
  if (mixer_saturated)       { flags |= TEL_FLAG_MIXER_SATURATED; }
  if (servo_released)        { flags |= TEL_FLAG_SERVO_RELEASED; }
  if (command->rf_loss_detected) { flags |= TEL_FLAG_RF_LOSS; }
  if (command->throttle_low) { flags |= TEL_FLAG_THROTTLE_LOW; }
  if (roll_pitch_dynamic)    { flags |= TEL_FLAG_RP_DYNAMIC; }
  if (mag_heading_valid)     { flags |= TEL_FLAG_MAG_HDG_VALID; }
  if (command->valid)        { flags |= TEL_FLAG_RC_VALID; }

  out->flags = flags;
  out->failsafe_mode = (uint8_t)failsafe_mode;
  out->imu_fail_streak = imu_fail_streak;
  out->mag_fail_streak = mag_fail_streak;

  out->roll_deg = attitude.roll_deg;
  out->pitch_deg = attitude.pitch_deg;
  out->heading_deg = heading_deg;
  out->heading_ref_deg = heading_hold_ref_deg;
  out->heading_error_deg = last_heading_error_deg;
  out->mag_heading_deg = mag_heading_deg;

  out->gyro_dps[0] = last_gyro_x_dps;
  out->gyro_dps[1] = last_gyro_y_dps;
  out->gyro_dps[2] = last_gyro_z_dps;
  out->pid_out[0] = last_roll_pid_output;
  out->pid_out[1] = last_pitch_pid_output;
  out->pid_out[2] = last_yaw_pid_output;

  for (uint8_t i = 0U; i < 4U; i++) {
    out->motor_us[i] = motor_output_us[i];
    out->rc_raw[i] = command->raw[i];
  }
  out->ch5_us = servo_ch5_pulse_us;
  out->servo_us = servo_output_us;

  out->roll_cmd = command->roll_cmd;
  out->pitch_cmd = command->pitch_cmd;
  out->yaw_cmd = command->yaw_cmd;
  out->throttle_cmd = command->throttle_norm;
  out->desired_yaw_rate = last_desired_yaw_rate;
  out->failsafe_throttle = failsafe_throttle_norm;
  out->tilt_boost = throttle_tilt_boost;

  out->accel_norm = accel_norm;
  out->accel_trust = accel_trust;
  out->mag_field = mag_field_strength;
  out->mag_raw[0] = mag_raw_x;
  out->mag_raw[1] = mag_raw_y;
  out->mag_raw[2] = mag_raw_z;

  out->yaw_dynamic = yaw_dynamic_damping_output;
  out->yaw_throttle_ff = yaw_throttle_feedforward_output;
  out->yaw_ff = yaw_feedforward_output;
  out->yaw_maneuver_bias = yaw_maneuver_bias_output;
  out->yaw_learn_bias = yaw_bias_output;
  out->mixer_yaw_scale = mixer_yaw_scale;
  out->mixer_rp_scale = mixer_rp_scale;
  out->yaw_maneuver_scale = yaw_maneuver_scale;
  out->takeoff_assist = takeoff_assist_scale;
  out->throttle_delta = throttle_delta_norm;
  out->throttle_rate = throttle_rate_norm_s;

  out->rc_center[0] = (uint16_t)rc_roll_center_runtime;
  out->rc_center[1] = (uint16_t)rc_pitch_center_runtime;
  out->rc_center[2] = (uint16_t)rc_yaw_center_runtime;

  out->pid_gain[0][0] = pid_roll.kp;  out->pid_gain[0][1] = pid_roll.ki;  out->pid_gain[0][2] = pid_roll.kd;
  out->pid_gain[1][0] = pid_pitch.kp; out->pid_gain[1][1] = pid_pitch.ki; out->pid_gain[1][2] = pid_pitch.kd;
  out->pid_gain[2][0] = pid_yaw.kp;   out->pid_gain[2][1] = pid_yaw.ki;   out->pid_gain[2][2] = pid_yaw.kd;

  out->altitude_m = current_baro_data.altitude_m;
  out->altitude_raw_m = current_baro_data.altitude_raw_m;
  out->climb_rate_ms = current_baro_data.climb_rate_ms;
  out->pressure_hpa = current_baro_data.pressure_hpa;
  out->baro_temp_c = current_baro_data.temperature_c;
  out->baro_ready = bmp388_is_ready();

  /* Du lieu dinh vi GPS (BZGNSS BZ-251 / USART3) */
  const GpsData *gps = gps_get_data();
  out->gps_lat               = gps->lat;
  out->gps_lon               = gps->lon;
  out->gps_alt_cm            = gps->alt_cm;
  out->gps_speed_cms         = gps->speed_cms;
  out->gps_course_cdeg       = gps->course_cdeg;
  out->gps_hdop              = gps->hdop_x100;
  out->gps_satellites        = gps->satellites;
  out->gps_fix_type          = gps->fix_type;
  out->gps_flags             = gps->flags;
  out->gps_home_dist_m       = gps->home_dist_m;
  out->gps_home_bearing_cdeg = gps->home_bearing_cdeg;
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  uint8_t index;
  uint32_t tim_channel;
  uint32_t capture;
  uint32_t pulse_width;
  uint32_t tick_ms;

  if (htim->Instance == TIM4) {
    if (htim->Channel != HAL_TIM_ACTIVE_CHANNEL_3) {
      return;
    }

    capture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
    if (servo_ch5_edge_state == 0U) {
      servo_ch5_rise_tick = capture;
      servo_ch5_edge_state = 1U;
      __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_FALLING);
      return;
    }

    if (capture >= servo_ch5_rise_tick) {
      pulse_width = capture - servo_ch5_rise_tick;
    } else {
      pulse_width = (65535U - servo_ch5_rise_tick) + capture + 1U;
    }

    servo_ch5_edge_state = 0U;
    __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_RISING);

    if ((pulse_width >= 800U) && (pulse_width <= 2200U)) {
      servo_ch5_pulse_us = (uint16_t)pulse_width;
      tick_ms = HAL_GetTick();
      servo_ch5_last_update_ms = (tick_ms == 0U) ? 1U : tick_ms;
    }
    return;
  }

  if (htim->Instance != TIM3) {
    return;
  }

  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
    index = 0U;
    tim_channel = TIM_CHANNEL_1;
  } else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
    index = 1U;
    tim_channel = TIM_CHANNEL_2;
  } else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
    index = 2U;
    tim_channel = TIM_CHANNEL_3;
  } else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) {
    index = 3U;
    tim_channel = TIM_CHANNEL_4;
  } else {
    return;
  }

  capture = HAL_TIM_ReadCapturedValue(htim, tim_channel);

  if (rc_edge_state[index] == 0U) {
    rc_rise_ticks[index] = capture;
    rc_edge_state[index] = 1U;
    __HAL_TIM_SET_CAPTUREPOLARITY(htim, tim_channel, TIM_INPUTCHANNELPOLARITY_FALLING);
    return;
  }

  if (capture >= rc_rise_ticks[index]) {
    pulse_width = capture - rc_rise_ticks[index];
  } else {
    pulse_width = (65535U - rc_rise_ticks[index]) + capture + 1U;
  }

  rc_edge_state[index] = 0U;
  __HAL_TIM_SET_CAPTUREPOLARITY(htim, tim_channel, TIM_INPUTCHANNELPOLARITY_RISING);

  if ((pulse_width >= 800U) && (pulse_width <= 2200U)) {
    rc_pulse_us[index] = (uint16_t)pulse_width;
    tick_ms = HAL_GetTick();
    rc_last_update_ms[index] = (tick_ms == 0U) ? 1U : tick_ms;
  }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  uint32_t last_loop_cycles;
  uint32_t next_loop_cycles;
  uint32_t loop_period_cycles;
  uint32_t last_log_ms = 0U;

  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();

  dwt_init();
  hw_i2c1_init();
  set_status_led(false);

  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK) {
    Error_Handler();
  }
  __HAL_TIM_MOE_ENABLE(&htim1);
  servo_grip();

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
  stop_all_motors();

  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_3);
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_4);
  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_3);

  uart_log("\r\n========================================\r\n");
  uart_log(" Blue Pill UAV Flight Controller\r\n");
  uart_log(" TIM2 motors / TIM3 receiver / TIM4 CH5 servo / ANGLE\r\n");
  uart_log(" Props OFF during setup and tests\r\n");
  uart_log("========================================\r\n");

  HAL_Delay(300U);
  imu_ready = mpu6050_init();
  if (imu_ready) {
    for (uint8_t retry = 0U; retry < 5U; retry++) {
      if (mpu6050_calibrate()) {
        imu_ready = true;
        break;
      }
      imu_ready = false;
      uart_log("[IMU] Calib rejected (motion). Retrying in 400ms... Keep board STILL!\r\n");
      HAL_Delay(400U);
    }
  }

  if (!imu_ready) {
    uart_log("[IMU] MPU6050 init/calibration failed after retries. Flight locked.\r\n");
  }

  mag_ready = imu_ready && mpu6050_enable_bypass() && hmc5883l_init();
  if (mag_ready) {
    uart_log("[MAG] HMC5883L ready.\r\n");
  } else {
    uart_log("[MAG] HMC5883L init failed. Heading hold disabled.\r\n");
  }

  /* Do thiet bi I2C va khoi tao BMP388 (chi doc du lieu de test phan cung) */
  i2c_scan();
  (void)bmp388_init();

  /* Telemetry nRF24: that bai KHONG khoa bay, chi tat telemetry (tu thu lai). */
  if (telemetry_init(telemetry_fill_cb)) {
    uart_log("[TEL] nRF24L01 ready (ch80, 1Mbps, 100 pkt/s).\r\n");
  } else {
    char tel_diag[128];
    uart_log("[TEL] nRF24L01 not found. Telemetry off, will retry.\r\n");
    telemetry_diag(tel_diag, sizeof(tel_diag));
    uart_log(tel_diag);
  }

  /* Khoi tao san GPS USART3 (PB10=TX, PB11=RX, 9600 baud) */
  gps_init();
  uart_log("[GPS] BZGNSS BZ-251 ready on USART3 (PB10=TX, PB11=RX, 9600 baud).\r\n");

  uart_log("[ESC] Holding 1000us for ESC boot.\r\n");
  HAL_Delay(2000U);
  uart_log("[ARM] Throttle low + yaw right hold 3s = arm\r\n");
  uart_log("[ARM] Throttle low + yaw left hold  3s = disarm\r\n");

  loop_period_cycles = SystemCoreClock / LOOP_HZ;
  last_loop_cycles = dwt_cycles32();
  next_loop_cycles = last_loop_cycles + loop_period_cycles;

  while (1) {
    const uint32_t now_cycles = dwt_cycles32();
    const uint32_t now_ms = HAL_GetTick();
    const uint32_t log_period_ms = armed ? LOG_PERIOD_ARMED_MS : LOG_PERIOD_DISARMED_MS;
    RcCommand command = {0};

    update_status_led();
    receiver_read_command(&command, now_ms);
    servo_update_from_ch5(now_ms);

    /* Cap nhat cam bien BMP388 chu ky 40ms (25Hz ODR) cho telemetry va log */
    if (bmp388_is_ready() && ((uint32_t)(now_ms - last_baro_sample_ms) >= 40U)) {
      last_baro_sample_ms = now_ms;
      (void)bmp388_read(&current_baro_data);
    }

    if ((uint32_t)(now_ms - last_log_ms) >= log_period_ms) {
      last_log_ms = now_ms;
      log_status(&command);

      /* In thong so BMP388 kiem tra phan cung khi chua ARM (chu ky 200ms) */
      if (!armed && bmp388_is_ready()) {
        char baro_str[104];
        (void)snprintf(baro_str, sizeof(baro_str),
                       "[BARO] T=%.1fC P=%.1fhPa Alt=%+.2fm (Raw=%+.2fm) Vz=%+.2fm/s\r\n",
                       current_baro_data.temperature_c,
                       current_baro_data.pressure_hpa,
                       current_baro_data.altitude_m,
                       current_baro_data.altitude_raw_m,
                       current_baro_data.climb_rate_ms);
        uart_log(baro_str);
      }

      /* In thong so GPS kiem tra khi co tin hieu BZ-251 tren USART3 */
      if (!armed && gps_get_data()->present) {
        const GpsData *g = gps_get_data();
        char gps_str[104];
        (void)snprintf(gps_str, sizeof(gps_str),
                       "[GPS] Fix=%u Sats=%u Lat=%.6f Lon=%.6f Alt=%.1fm Spd=%.1fkm/h HDOP=%.2f\r\n",
                       g->fix_type,
                       g->satellites,
                       (double)g->lat / 1e7,
                       (double)g->lon / 1e7,
                       (double)g->alt_cm / 100.0,
                       (double)g->speed_cms * 0.036,
                       (double)g->hdop_x100 / 100.0);
        uart_log(gps_str);
      }
    }

    if ((int32_t)(now_cycles - next_loop_cycles) < 0) {
      /* Thoi gian ranh truoc nhip dieu khien ke tiep: telemetry va GPS chi chay neu con du slack. */
      telemetry_poll(now_ms, (int32_t)(next_loop_cycles - dwt_cycles32()), &command);
      gps_poll(now_ms);
      continue;
    }

    while ((int32_t)(now_cycles - next_loop_cycles) >= 0) {
      next_loop_cycles += loop_period_cycles;
    }

    {
      const float dt = (float)((uint32_t)(now_cycles - last_loop_cycles)) / (float)SystemCoreClock;
      ImuSample sample = {0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f};
      RcCommand control_command = {0};
      bool sensors_ok = imu_ready;
      bool sample_valid = false;

      telemetry_note_loop_cycles((uint32_t)(now_cycles - last_loop_cycles));
      last_loop_cycles = now_cycles;

      if (dt <= 0.0f || dt > 0.02f) {
        stop_all_motors();
        reset_all_pids();
        next_loop_cycles = now_cycles + loop_period_cycles;
        continue;
      }

      if (imu_ready) {
        if (mpu6050_read_filtered(&sample, dt)) {
          imu_fail_streak = 0U;
          sample_valid = true;
          update_attitude(&sample, dt);
          update_aux_sensors(now_ms);
        } else {
          imu_fail_streak++;
          if (imu_fail_streak >= IMU_FAIL_LIMIT) {
            imu_ready = false;
            sensors_ok = false;
            uart_log("[IMU] Repeated read failure. Flight locked.\r\n");
          }
        }
      } else {
        sensors_ok = false;
      }

      update_failsafe_state(&command, sensors_ok, &sample, sample_valid, now_ms, dt);
      update_arming_state(&command, sensors_ok, now_ms);

      if (!sensors_ok) {
        stop_all_motors();
        continue;
      }

      if (!sample_valid) {
        stop_all_motors();
        continue;
      }

      build_control_command(&command, &control_command, now_ms, dt);

      if (!control_command.valid) {
        stop_all_motors();
        continue;
      }

      run_flight_control(&control_command, &sample, dt);
    }
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71U;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 19999U;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0U;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = SERVO_GRIP_US;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim1);
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71U;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2499U;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = MOTOR_STOP_US;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim2);
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 71U;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535U;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK) {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 8U;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_2) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_3) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_4) != HAL_OK) {
    Error_Handler();
  }
}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 71U;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535U;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK) {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 8U;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_3) != HAL_OK) {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200U;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK) {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_SET);

  GPIO_InitStruct.Pin = STATUS_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(STATUS_LED_GPIO_Port, &GPIO_InitStruct);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1) {
    HAL_GPIO_TogglePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin);
    for (volatile uint32_t delay = 0U; delay < 300000U; delay++) {
      __NOP();
    }
  }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
