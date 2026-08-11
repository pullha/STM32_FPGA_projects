#ifndef ELEVATOR_MOTOR_H
#define ELEVATOR_MOTOR_H

#include <stdint.h>

#include "elevator_algorithm_common.h"

#define ELEVATOR_MOTOR_DEFAULT_STEP_DELAY_MS  3U
#define ELEVATOR_MOTOR_DEFAULT_STEP_DELAY_US  (ELEVATOR_MOTOR_DEFAULT_STEP_DELAY_MS * 1000U)

/* Result values returned by motor movement helpers. */
typedef enum
{
  ELEVATOR_MOTOR_STATUS_OK = 0,
  ELEVATOR_MOTOR_STATUS_BAD_ARGUMENT,
  ELEVATOR_MOTOR_STATUS_TARGET_REACHED,
  ELEVATOR_MOTOR_STATUS_MAX_STEPS_REACHED
} ElevatorMotorStatus;

/* Turn off every motor coil output for both elevator cars. */
void ElevatorMotor_AllOff(void);

/* Turn off every motor coil output for one elevator car. */
void ElevatorMotor_CarOff(ElevatorCarId car);

/* Move one half-step in elevator direction. UP uses reverse motor rotation; DOWN uses forward rotation. */
void ElevatorMotor_Step(ElevatorCarId car, ElevatorDirection direction);

/* Move a fixed number of half-steps with a delay between each step. */
void ElevatorMotor_MoveSteps(ElevatorCarId car,
                             ElevatorDirection direction,
                             uint32_t steps,
                             uint16_t step_delay_ms);

/* Move a fixed number of half-steps with a microsecond delay between each step. */
void ElevatorMotor_MoveStepsUs(ElevatorCarId car,
                               ElevatorDirection direction,
                               uint32_t steps,
                               uint32_t step_delay_us);

/* Move until the target floor photo interrupter is blocked, or until max_steps is reached. */
ElevatorMotorStatus ElevatorMotor_MoveUntilFloorSensor(ElevatorCarId car,
                                                       ElevatorDirection direction,
                                                       uint8_t target_floor,
                                                       uint32_t max_steps,
                                                       uint32_t step_delay_us);

#endif /* ELEVATOR_MOTOR_H */
