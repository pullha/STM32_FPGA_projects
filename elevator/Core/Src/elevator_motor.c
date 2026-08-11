#include "elevator_motor.h"

#include "elevator_floor_sensor.h"
#include "main.h"

typedef struct
{
  GPIO_TypeDef *port;
  uint16_t pin;
} MotorPin;

/* Motor GPIO map. Each car has four ULN2003 inputs connected to IN1~IN4. */
static const MotorPin MotorPins[ELEVATOR_CAR_COUNT][4U] =
{
  {
    { MOTOR_A_IN1_GPIO_Port, MOTOR_A_IN1_Pin },
    { MOTOR_A_IN2_GPIO_Port, MOTOR_A_IN2_Pin },
    { MOTOR_A_IN3_GPIO_Port, MOTOR_A_IN3_Pin },
    { MOTOR_A_IN4_GPIO_Port, MOTOR_A_IN4_Pin }
  },
  {
    { MOTOR_B_IN1_GPIO_Port, MOTOR_B_IN1_Pin },
    { MOTOR_B_IN2_GPIO_Port, MOTOR_B_IN2_Pin },
    { MOTOR_B_IN3_GPIO_Port, MOTOR_B_IN3_Pin },
    { MOTOR_B_IN4_GPIO_Port, MOTOR_B_IN4_Pin }
  }
};

/* 28BYJ-48 half-step pattern. Reversing the sequence reverses motor direction. */
static const uint8_t HalfStepPattern[8U][4U] =
{
  { 1U, 0U, 0U, 0U },
  { 1U, 1U, 0U, 0U },
  { 0U, 1U, 0U, 0U },
  { 0U, 1U, 1U, 0U },
  { 0U, 0U, 1U, 0U },
  { 0U, 0U, 1U, 1U },
  { 0U, 0U, 0U, 1U },
  { 1U, 0U, 0U, 1U }
};

/* Current phase index for each motor so consecutive calls continue smoothly. */
static uint8_t MotorPhaseIndex[ELEVATOR_CAR_COUNT] = { 0U, 0U };

/* Guard helper so invalid enum values do not index the motor table. */
static bool CarIsValid(ElevatorCarId car)
{
  return (car == ELEVATOR_CAR_A) || (car == ELEVATOR_CAR_B);
}

/* Enable the Cortex-M cycle counter used for sub-millisecond motor delays. */
static void EnableCycleCounter(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/* Busy-wait delay in microseconds. This is used only for motor step timing. */
static void DelayUs(uint32_t delay_us)
{
  uint32_t cycles;
  uint32_t start_cycle;

  if (delay_us == 0U)
  {
    return;
  }

  EnableCycleCounter();
  cycles = (SystemCoreClock / 1000000U) * delay_us;
  start_cycle = DWT->CYCCNT;

  while ((DWT->CYCCNT - start_cycle) < cycles)
  {
  }
}

/* Apply one half-step pattern to the four ULN2003 input pins. */
static void SetPhase(ElevatorCarId car, uint8_t phase_index)
{
  uint8_t coil_index;

  for (coil_index = 0U; coil_index < 4U; coil_index++)
  {
    HAL_GPIO_WritePin(MotorPins[car][coil_index].port,
                      MotorPins[car][coil_index].pin,
                      HalfStepPattern[phase_index][coil_index] ? GPIO_PIN_SET : GPIO_PIN_RESET);
  }
}

/* Release both motors. This avoids holding coil current while idle. */
void ElevatorMotor_AllOff(void)
{
  ElevatorMotor_CarOff(ELEVATOR_CAR_A);
  ElevatorMotor_CarOff(ELEVATOR_CAR_B);
}

/* Release one motor by driving every coil input low. */
void ElevatorMotor_CarOff(ElevatorCarId car)
{
  uint8_t coil_index;

  if (!CarIsValid(car))
  {
    return;
  }

  for (coil_index = 0U; coil_index < 4U; coil_index++)
  {
    HAL_GPIO_WritePin(MotorPins[car][coil_index].port, MotorPins[car][coil_index].pin, GPIO_PIN_RESET);
  }
}

/* Move exactly one half-step.
 * Physical wiring: reverse motor rotation lifts the elevator, forward rotation lowers it.
 */
void ElevatorMotor_Step(ElevatorCarId car, ElevatorDirection direction)
{
  if (!CarIsValid(car) || (direction == ELEVATOR_DIR_IDLE))
  {
    return;
  }

  if (direction == ELEVATOR_DIR_UP)
  {
    MotorPhaseIndex[car] = (uint8_t)((MotorPhaseIndex[car] + 7U) % 8U);
  }
  else
  {
    MotorPhaseIndex[car] = (uint8_t)((MotorPhaseIndex[car] + 1U) % 8U);
  }

  SetPhase(car, MotorPhaseIndex[car]);
}

/* Blocking fixed-step movement. Good for first motor tests before adding distance feedback. */
void ElevatorMotor_MoveSteps(ElevatorCarId car,
                             ElevatorDirection direction,
                             uint32_t steps,
                             uint16_t step_delay_ms)
{
  uint16_t active_step_delay_ms = (step_delay_ms == 0U) ? ELEVATOR_MOTOR_DEFAULT_STEP_DELAY_MS : step_delay_ms;

  ElevatorMotor_MoveStepsUs(car, direction, steps, (uint32_t)active_step_delay_ms * 1000U);
}

/* Blocking fixed-step movement with microsecond timing for speed tests. */
void ElevatorMotor_MoveStepsUs(ElevatorCarId car,
                               ElevatorDirection direction,
                               uint32_t steps,
                               uint32_t step_delay_us)
{
  uint32_t step_index;
  uint32_t active_step_delay_us = (step_delay_us == 0U) ? ELEVATOR_MOTOR_DEFAULT_STEP_DELAY_US : step_delay_us;

  if (!CarIsValid(car) || (direction == ELEVATOR_DIR_IDLE))
  {
    return;
  }

  for (step_index = 0U; step_index < steps; step_index++)
  {
    ElevatorMotor_Step(car, direction);
    DelayUs(active_step_delay_us);
  }

  ElevatorMotor_CarOff(car);
}

/* Blocking movement with floor-sensor feedback.
 * The car stops as soon as the target floor photo interrupter is debounced as blocked.
 */
ElevatorMotorStatus ElevatorMotor_MoveUntilFloorSensor(ElevatorCarId car,
                                                       ElevatorDirection direction,
                                                       uint8_t target_floor,
                                                       uint32_t max_steps,
                                                       uint32_t step_delay_us)
{
  uint32_t step_index;
  uint32_t active_step_delay_us = (step_delay_us == 0U) ? ELEVATOR_MOTOR_DEFAULT_STEP_DELAY_US : step_delay_us;

  if (!CarIsValid(car) ||
      (direction == ELEVATOR_DIR_IDLE) ||
      !ElevatorFloorIsValid(target_floor))
  {
    return ELEVATOR_MOTOR_STATUS_BAD_ARGUMENT;
  }

  if (ElevatorFloorSensor_IsArrived(car, target_floor))
  {
    ElevatorMotor_CarOff(car);
    return ELEVATOR_MOTOR_STATUS_TARGET_REACHED;
  }

  for (step_index = 0U; step_index < max_steps; step_index++)
  {
    ElevatorMotor_Step(car, direction);
    DelayUs(active_step_delay_us);

    if (ElevatorFloorSensor_IsBlocked(car, target_floor) &&
        ElevatorFloorSensor_IsArrived(car, target_floor))
    {
      ElevatorMotor_CarOff(car);
      return ELEVATOR_MOTOR_STATUS_TARGET_REACHED;
    }
  }

  ElevatorMotor_CarOff(car);
  return ELEVATOR_MOTOR_STATUS_MAX_STEPS_REACHED;
}
