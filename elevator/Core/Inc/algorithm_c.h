#ifndef ALGORITHM_C_H
#define ALGORITHM_C_H

#include "elevator_algorithm_common.h"

/* Algorithm C: nearest car with direction priority.
 * A moving car is preferred when it is already traveling in the call direction
 * and will pass the call floor. Idle cars are considered next.
 */
ElevatorAssignment ElevatorAlgorithmC_Assign(const ElevatorState elevators[ELEVATOR_CAR_COUNT],
                                             const ElevatorRequest *request);

/* Assign several requests in order and add accepted requests to each car's stop list. */
void ElevatorAlgorithmC_AssignBatch(ElevatorState elevators[ELEVATOR_CAR_COUNT],
                                    const ElevatorRequest *requests,
                                    uint8_t request_count,
                                    ElevatorAssignment *assignments);

#endif /* ALGORITHM_C_H */
