#include "moter.h"
#include "tim.h"

#define MOTER_MAX_PERCENT 100
#define MOTER_MIN_PERCENT (-100)
#define MOTER_MAX_PERCENT_X10 1000
#define MOTER_MIN_PERCENT_X10 (-1000)

/* Straight PID gains are scaled by 1000 to avoid floating point in FreeRTOS. */
#define MOTER_STRAIGHT_PID_KP_X1000 500
#define MOTER_STRAIGHT_PID_KI_X1000 20
#define MOTER_STRAIGHT_PID_KD_X1000 100
#define MOTER_STRAIGHT_PID_INTEGRAL_LIMIT 400
#define MOTER_STRAIGHT_PID_MAX_CORRECTION_X10 60

/* Right motor calibration found during floor testing: left 50%, right 48%. */
#define MOTER_RIGHT_STRAIGHT_SCALE_X1000 960

static int16_t Moter_ClampPercent(int16_t percent);
static int16_t Moter_ClampPositivePercent(int16_t percent);
static int16_t Moter_ClampPercentX10(int16_t percent_x10);
static int16_t Moter_GetCalibratedRightPercent(int16_t left_percent);
static uint32_t Moter_PercentX10ToPulse(int16_t percent_x10);
static void Moter_SetSpeedX10(MoterSide side, int16_t percent_x10);
static void Moter_SetLeftDirection(int16_t percent_x10);
static void Moter_SetRightDirection(int16_t percent_x10);
static uint32_t Moter_ReadLeftEncoderRaw(void);
static uint16_t Moter_ReadRightEncoderRaw(void);
static int32_t Moter_LimitInt32(int32_t value, int32_t min_value, int32_t max_value);

/* Active timed command state. This is the bridge for future CAN commands. */
static MoterCommand active_command = MOTER_COMMAND_STOP;
static uint8_t command_running = 0;
static uint32_t command_start_tick = 0;
static uint32_t command_duration_ms = 0;

/* Straight PID state. Values ending in x10 store percent * 10. */
static int16_t straight_base_left_x10 = 0;
static int16_t straight_base_right_x10 = 0;
static int16_t straight_left_command_x10 = 0;
static int16_t straight_right_command_x10 = 0;
static uint32_t straight_previous_left_raw = 0;
static uint16_t straight_previous_right_raw = 0;
static int32_t straight_error = 0;
static int32_t straight_previous_error = 0;
static int32_t straight_integral = 0;
static int32_t straight_left_delta = 0;
static int32_t straight_right_delta = 0;
static uint8_t straight_pid_enabled = 0;

void Moter_Init(void)
{
  /* TIM3 drives L298 EN pins, TIM1/TIM2 count quadrature encoder pulses. */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

  Moter_ResetEncoders();
  Moter_Stop();
}

void Moter_Stop(void)
{
  /* Stop must clear both the command runner and PID before disabling outputs. */
  active_command = MOTER_COMMAND_STOP;
  command_running = 0;
  command_start_tick = 0;
  command_duration_ms = 0;
  straight_pid_enabled = 0;

  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);

  HAL_GPIO_WritePin(L298_IN1_GPIO_Port, L298_IN1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(L298_IN2_GPIO_Port, L298_IN2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(L298_IN3_GPIO_Port, L298_IN3_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(L298_IN4_GPIO_Port, L298_IN4_Pin, GPIO_PIN_RESET);
}

void Moter_SetSpeed(MoterSide side, int16_t percent)
{
  percent = Moter_ClampPercent(percent);
  Moter_SetSpeedX10(side, (int16_t)(percent * 10));
}

void Moter_SetBothSpeed(int16_t left_percent, int16_t right_percent)
{
  Moter_SetSpeed(MOTER_LEFT, left_percent);
  Moter_SetSpeed(MOTER_RIGHT, right_percent);
}

void Moter_MoveForward(int16_t percent)
{
  int16_t left_percent = Moter_ClampPositivePercent(percent);
  int16_t right_percent = Moter_GetCalibratedRightPercent(left_percent);

  /* Use PID for straight motion because wheel speed is not perfectly matched. */
  Moter_StraightPidStart(left_percent, right_percent);
}

void Moter_MoveBackward(int16_t percent)
{
  int16_t left_percent = (int16_t)-Moter_ClampPositivePercent(percent);
  int16_t right_percent = Moter_GetCalibratedRightPercent(left_percent);

  Moter_StraightPidStart(left_percent, right_percent);
}

void Moter_TurnLeft(int16_t percent)
{
  int16_t turn_percent = Moter_ClampPositivePercent(percent);

  /* Pivot turn: left wheel backward, right wheel forward. */
  straight_pid_enabled = 0;
  Moter_SetBothSpeed((int16_t)-turn_percent, turn_percent);
}

void Moter_TurnRight(int16_t percent)
{
  int16_t turn_percent = Moter_ClampPositivePercent(percent);

  /* Pivot turn: left wheel forward, right wheel backward. */
  straight_pid_enabled = 0;
  Moter_SetBothSpeed(turn_percent, (int16_t)-turn_percent);
}

void Moter_MoveUpdate(void)
{
  Moter_StraightPidUpdate();
}

void Moter_CommandStart(MoterCommand command, int16_t percent, uint32_t duration_ms)
{
  int16_t command_percent = Moter_ClampPositivePercent(percent);

  /* A zero-duration or zero-speed command is treated as an immediate stop. */
  if ((command == MOTER_COMMAND_STOP) || (command_percent == 0) || (duration_ms == 0U))
  {
    Moter_Stop();
    return;
  }

  switch (command)
  {
    case MOTER_COMMAND_FORWARD:
      Moter_MoveForward(command_percent);
      break;

    case MOTER_COMMAND_BACKWARD:
      Moter_MoveBackward(command_percent);
      break;

    case MOTER_COMMAND_TURN_LEFT:
      Moter_TurnLeft(command_percent);
      break;

    case MOTER_COMMAND_TURN_RIGHT:
      Moter_TurnRight(command_percent);
      break;

    default:
      Moter_Stop();
      return;
  }

  active_command = command;
  command_running = 1;
  command_start_tick = HAL_GetTick();
  command_duration_ms = duration_ms;
}

void Moter_CommandUpdate(void)
{
  uint32_t elapsed_ms;

  if (command_running == 0U)
  {
    return;
  }

  if ((active_command == MOTER_COMMAND_FORWARD) || (active_command == MOTER_COMMAND_BACKWARD))
  {
    /* Straight commands need periodic encoder correction while they run. */
    Moter_MoveUpdate();
  }

  elapsed_ms = HAL_GetTick() - command_start_tick;
  if (elapsed_ms >= command_duration_ms)
  {
    Moter_Stop();
  }
}

uint8_t Moter_CommandIsRunning(void)
{
  return command_running;
}

MoterCommand Moter_GetActiveCommand(void)
{
  return active_command;
}

uint32_t Moter_GetCommandRemainingMs(void)
{
  uint32_t elapsed_ms;

  if (command_running == 0U)
  {
    return 0;
  }

  elapsed_ms = HAL_GetTick() - command_start_tick;
  if (elapsed_ms >= command_duration_ms)
  {
    return 0;
  }

  return command_duration_ms - elapsed_ms;
}

int32_t Moter_GetLeftEncoder(void)
{
  return (int32_t)Moter_ReadLeftEncoderRaw();
}

int32_t Moter_GetRightEncoder(void)
{
  /* The right encoder phase is reversed on this chassis, so normalize it here. */
  return -(int32_t)(int16_t)Moter_ReadRightEncoderRaw();
}

void Moter_ResetEncoders(void)
{
  __HAL_TIM_SET_COUNTER(&htim1, 0);
  __HAL_TIM_SET_COUNTER(&htim2, 0);
}

void Moter_StraightPidStart(int16_t base_left_percent, int16_t base_right_percent)
{
  straight_base_left_x10 = Moter_ClampPercentX10((int16_t)(base_left_percent * 10));
  straight_base_right_x10 = Moter_ClampPercentX10((int16_t)(base_right_percent * 10));
  straight_left_command_x10 = straight_base_left_x10;
  straight_right_command_x10 = straight_base_right_x10;

  Moter_ResetEncoders();

  /* Store the first raw counter values so the next update can use deltas. */
  straight_previous_left_raw = Moter_ReadLeftEncoderRaw();
  straight_previous_right_raw = Moter_ReadRightEncoderRaw();
  straight_error = 0;
  straight_previous_error = 0;
  straight_integral = 0;
  straight_left_delta = 0;
  straight_right_delta = 0;
  straight_pid_enabled = 1;

  Moter_SetSpeedX10(MOTER_LEFT, straight_left_command_x10);
  Moter_SetSpeedX10(MOTER_RIGHT, straight_right_command_x10);
}

void Moter_StraightPidUpdate(void)
{
  uint32_t current_left_raw;
  uint16_t current_right_raw;
  uint16_t right_raw_delta;
  int32_t derivative;
  int32_t correction_x10;

  if (straight_pid_enabled == 0U)
  {
    return;
  }

  current_left_raw = Moter_ReadLeftEncoderRaw();
  current_right_raw = Moter_ReadRightEncoderRaw();

  /* Use delta counts per update period instead of total counts. */
  straight_left_delta = (int32_t)(current_left_raw - straight_previous_left_raw);
  right_raw_delta = (uint16_t)(current_right_raw - straight_previous_right_raw);
  straight_right_delta = -(int32_t)(int16_t)right_raw_delta;

  straight_previous_left_raw = current_left_raw;
  straight_previous_right_raw = current_right_raw;

  straight_error = straight_left_delta - straight_right_delta;
  straight_integral += straight_error;
  straight_integral = Moter_LimitInt32(straight_integral,
                                       -MOTER_STRAIGHT_PID_INTEGRAL_LIMIT,
                                       MOTER_STRAIGHT_PID_INTEGRAL_LIMIT);

  derivative = straight_error - straight_previous_error;
  straight_previous_error = straight_error;

  /* Positive correction slows the left wheel and speeds up the right wheel. */
  correction_x10 = ((MOTER_STRAIGHT_PID_KP_X1000 * straight_error) +
                    (MOTER_STRAIGHT_PID_KI_X1000 * straight_integral) +
                    (MOTER_STRAIGHT_PID_KD_X1000 * derivative)) / 1000;
  correction_x10 = Moter_LimitInt32(correction_x10,
                                    -MOTER_STRAIGHT_PID_MAX_CORRECTION_X10,
                                    MOTER_STRAIGHT_PID_MAX_CORRECTION_X10);

  straight_left_command_x10 = Moter_ClampPercentX10((int16_t)(straight_base_left_x10 - correction_x10));
  straight_right_command_x10 = Moter_ClampPercentX10((int16_t)(straight_base_right_x10 + correction_x10));

  Moter_SetSpeedX10(MOTER_LEFT, straight_left_command_x10);
  Moter_SetSpeedX10(MOTER_RIGHT, straight_right_command_x10);
}

int32_t Moter_GetStraightPidError(void)
{
  return straight_error;
}

int32_t Moter_GetStraightPidLeftDelta(void)
{
  return straight_left_delta;
}

int32_t Moter_GetStraightPidRightDelta(void)
{
  return straight_right_delta;
}

int16_t Moter_GetLeftCommandPercentX10(void)
{
  return straight_left_command_x10;
}

int16_t Moter_GetRightCommandPercentX10(void)
{
  return straight_right_command_x10;
}

static int16_t Moter_ClampPercent(int16_t percent)
{
  if (percent > MOTER_MAX_PERCENT)
  {
    return MOTER_MAX_PERCENT;
  }

  if (percent < MOTER_MIN_PERCENT)
  {
    return MOTER_MIN_PERCENT;
  }

  return percent;
}

static int16_t Moter_ClampPositivePercent(int16_t percent)
{
  if (percent < 0)
  {
    percent = (int16_t)-percent;
  }

  if (percent > MOTER_MAX_PERCENT)
  {
    return MOTER_MAX_PERCENT;
  }

  return percent;
}

static int16_t Moter_ClampPercentX10(int16_t percent_x10)
{
  if (percent_x10 > MOTER_MAX_PERCENT_X10)
  {
    return MOTER_MAX_PERCENT_X10;
  }

  if (percent_x10 < MOTER_MIN_PERCENT_X10)
  {
    return MOTER_MIN_PERCENT_X10;
  }

  return percent_x10;
}

static int16_t Moter_GetCalibratedRightPercent(int16_t left_percent)
{
  /* Applies the measured right-wheel scale before PID starts correcting. */
  return (int16_t)((left_percent * MOTER_RIGHT_STRAIGHT_SCALE_X1000) / 1000);
}

static uint32_t Moter_PercentX10ToPulse(int16_t percent_x10)
{
  uint32_t abs_percent_x10 = (percent_x10 < 0) ? (uint32_t)(-percent_x10) : (uint32_t)percent_x10;
  uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim3);

  return (period * abs_percent_x10) / 1000U;
}

static void Moter_SetSpeedX10(MoterSide side, int16_t percent_x10)
{
  percent_x10 = Moter_ClampPercentX10(percent_x10);

  if (side == MOTER_LEFT)
  {
    Moter_SetLeftDirection(percent_x10);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, Moter_PercentX10ToPulse(percent_x10));
  }
  else
  {
    Moter_SetRightDirection(percent_x10);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, Moter_PercentX10ToPulse(percent_x10));
  }
}

static void Moter_SetLeftDirection(int16_t percent_x10)
{
  if (percent_x10 > 0)
  {
    HAL_GPIO_WritePin(L298_IN1_GPIO_Port, L298_IN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(L298_IN2_GPIO_Port, L298_IN2_Pin, GPIO_PIN_RESET);
  }
  else if (percent_x10 < 0)
  {
    HAL_GPIO_WritePin(L298_IN1_GPIO_Port, L298_IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(L298_IN2_GPIO_Port, L298_IN2_Pin, GPIO_PIN_SET);
  }
  else
  {
    HAL_GPIO_WritePin(L298_IN1_GPIO_Port, L298_IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(L298_IN2_GPIO_Port, L298_IN2_Pin, GPIO_PIN_RESET);
  }
}

static void Moter_SetRightDirection(int16_t percent_x10)
{
  if (percent_x10 > 0)
  {
    HAL_GPIO_WritePin(L298_IN3_GPIO_Port, L298_IN3_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(L298_IN4_GPIO_Port, L298_IN4_Pin, GPIO_PIN_RESET);
  }
  else if (percent_x10 < 0)
  {
    HAL_GPIO_WritePin(L298_IN3_GPIO_Port, L298_IN3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(L298_IN4_GPIO_Port, L298_IN4_Pin, GPIO_PIN_SET);
  }
  else
  {
    HAL_GPIO_WritePin(L298_IN3_GPIO_Port, L298_IN3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(L298_IN4_GPIO_Port, L298_IN4_Pin, GPIO_PIN_RESET);
  }
}

static uint32_t Moter_ReadLeftEncoderRaw(void)
{
  return __HAL_TIM_GET_COUNTER(&htim2);
}

static uint16_t Moter_ReadRightEncoderRaw(void)
{
  return (uint16_t)__HAL_TIM_GET_COUNTER(&htim1);
}

static int32_t Moter_LimitInt32(int32_t value, int32_t min_value, int32_t max_value)
{
  if (value > max_value)
  {
    return max_value;
  }

  if (value < min_value)
  {
    return min_value;
  }

  return value;
}
