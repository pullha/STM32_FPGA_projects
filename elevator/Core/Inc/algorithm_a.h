#ifndef ALGORITHM_A_H
#define ALGORITHM_A_H

#include "elevator_algorithm_common.h"

/* Algorithm A: fixed odd/even service zones.
 * Car A serves floors 1, 3, and 5. Car B serves floors 1, 2, and 4.
 * On floor 1, the scenario can explicitly request car A or car B.
 */
ElevatorAssignment ElevatorAlgorithmA_Assign(const ElevatorState elevators[ELEVATOR_CAR_COUNT],
                                             const ElevatorRequest *request);

/* Assign several requests in order and add accepted requests to each car's stop list. */
void ElevatorAlgorithmA_AssignBatch(ElevatorState elevators[ELEVATOR_CAR_COUNT],
                                    const ElevatorRequest *requests,
                                    uint8_t request_count,
                                    ElevatorAssignment *assignments);

#endif /* ALGORITHM_A_H */
