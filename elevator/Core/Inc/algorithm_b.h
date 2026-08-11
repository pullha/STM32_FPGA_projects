#ifndef ALGORITHM_B_H
#define ALGORITHM_B_H

#include "elevator_algorithm_common.h"

/* Algorithm B: fixed low/high service zones.
 * Car A serves floors 1, 2, and 3. Car B serves floors 3, 4, and 5.
 * On floor 3, DOWN calls go to A and UP calls go to B.
 */
ElevatorAssignment ElevatorAlgorithmB_Assign(const ElevatorState elevators[ELEVATOR_CAR_COUNT],
                                             const ElevatorRequest *request);

/* Assign several requests in order and add accepted requests to each car's stop list. */
void ElevatorAlgorithmB_AssignBatch(ElevatorState elevators[ELEVATOR_CAR_COUNT],
                                    const ElevatorRequest *requests,
                                    uint8_t request_count,
                                    ElevatorAssignment *assignments);

#endif /* ALGORITHM_B_H */
