#ifndef ELEVATOR_LED_H
#define ELEVATOR_LED_H

#include <stdbool.h>
#include <stdint.h>

#include "elevator_algorithm_common.h"

/* Turn off every floor LED for both elevator cars. */
void ElevatorLed_AllOff(void);

/* Turn off all floor LEDs for one elevator car. */
void ElevatorLed_CarOff(ElevatorCarId car);

/* Set one floor LED on or off. The floor number is 1 to 5. */
bool ElevatorLed_Set(ElevatorCarId car, uint8_t floor, bool on);

/* Show one stopped floor: all LEDs for the car are turned off, then one floor LED is turned on. */
bool ElevatorLed_ShowFloor(ElevatorCarId car, uint8_t floor);

#endif /* ELEVATOR_LED_H */
