#ifndef ELEVATOR_FLOOR_SENSOR_H
#define ELEVATOR_FLOOR_SENSOR_H

#include <stdbool.h>
#include <stdint.h>

#include "elevator_algorithm_common.h"

/* Photo interrupter signals are active-high: clear beam = LOW, blocked beam = HIGH. */
#define ELEVATOR_FLOOR_SENSOR_DEBOUNCE_MS  30U

/* Configure the GPIO pins used by the ten floor photo interrupters. */
void ElevatorFloorSensor_Init(void);

/* Record a GPIO EXTI event. Call this from HAL_GPIO_EXTI_Callback(). */
void ElevatorFloorSensor_HandleExti(uint16_t gpio_pin);

/* Return true when the selected floor sensor is currently blocked. */
bool ElevatorFloorSensor_IsBlocked(ElevatorCarId car, uint8_t floor);

/* Return true when the selected floor sensor stays blocked through the debounce window. */
bool ElevatorFloorSensor_IsArrived(ElevatorCarId car, uint8_t floor);

/* Return a bit mask of blocked sensors. Bit 0 is floor 1, bit 4 is floor 5. */
uint16_t ElevatorFloorSensor_ReadMask(ElevatorCarId car);

/* Return one active floor number, or 0 when no floor or more than one floor is active. */
uint8_t ElevatorFloorSensor_GetActiveFloor(ElevatorCarId car);

/* Return the floor bits that have generated EXTI events since startup. */
uint16_t ElevatorFloorSensor_GetInterruptMask(ElevatorCarId car);

/* Return the number of accepted EXTI events since startup. */
uint32_t ElevatorFloorSensor_GetInterruptCount(ElevatorCarId car);

#endif /* ELEVATOR_FLOOR_SENSOR_H */
