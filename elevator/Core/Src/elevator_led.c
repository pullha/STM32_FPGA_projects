#include "elevator_led.h"

#include "main.h"

typedef struct
{
  GPIO_TypeDef *port;
  uint16_t pin;
} LedPin;

/* LED GPIO map. Index 0 is car A, index 1 is car B. Floor index is floor - 1. */
static const LedPin LedPins[ELEVATOR_CAR_COUNT][ELEVATOR_FLOOR_MAX] =
{
  {
    { LED_A_F1_GPIO_Port, LED_A_F1_Pin },
    { LED_A_F2_GPIO_Port, LED_A_F2_Pin },
    { LED_A_F3_GPIO_Port, LED_A_F3_Pin },
    { LED_A_F4_GPIO_Port, LED_A_F4_Pin },
    { LED_A_F5_GPIO_Port, LED_A_F5_Pin }
  },
  {
    { LED_B_F1_GPIO_Port, LED_B_F1_Pin },
    { LED_B_F2_GPIO_Port, LED_B_F2_Pin },
    { LED_B_F3_GPIO_Port, LED_B_F3_Pin },
    { LED_B_F4_GPIO_Port, LED_B_F4_Pin },
    { LED_B_F5_GPIO_Port, LED_B_F5_Pin }
  }
};

/* Guard helper so invalid enum values do not index the LED table. */
static bool CarIsValid(ElevatorCarId car)
{
  return (car == ELEVATOR_CAR_A) || (car == ELEVATOR_CAR_B);
}

/* Used before movement starts or when resetting the visual state. */
void ElevatorLed_AllOff(void)
{
  ElevatorLed_CarOff(ELEVATOR_CAR_A);
  ElevatorLed_CarOff(ELEVATOR_CAR_B);
}

/* Clear all five floor LEDs for one elevator. */
void ElevatorLed_CarOff(ElevatorCarId car)
{
  uint8_t floor_index;

  if (!CarIsValid(car))
  {
    return;
  }

  for (floor_index = 0U; floor_index < ELEVATOR_FLOOR_MAX; floor_index++)
  {
    HAL_GPIO_WritePin(LedPins[car][floor_index].port, LedPins[car][floor_index].pin, GPIO_PIN_RESET);
  }
}

/* Directly control a single LED. This is useful for simple LED hardware tests. */
bool ElevatorLed_Set(ElevatorCarId car, uint8_t floor, bool on)
{
  if (!CarIsValid(car) || !ElevatorFloorIsValid(floor))
  {
    return false;
  }

  HAL_GPIO_WritePin(LedPins[car][floor - 1U].port,
                    LedPins[car][floor - 1U].pin,
                    on ? GPIO_PIN_SET : GPIO_PIN_RESET);

  return true;
}

/* Show the floor where the elevator is currently stopped. */
bool ElevatorLed_ShowFloor(ElevatorCarId car, uint8_t floor)
{
  if (!CarIsValid(car) || !ElevatorFloorIsValid(floor))
  {
    return false;
  }

  ElevatorLed_CarOff(car);
  return ElevatorLed_Set(car, floor, true);
}
