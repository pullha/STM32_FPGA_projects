#ifndef ALGORITHM_D_H
#define ALGORITHM_D_H

#include "elevator_algorithm_common.h"

/* Algorithm D: ETA-based dispatch.
 * Existing stop reservations, travel time per floor, and stop/dwell time are
 * used to estimate which car reaches the call floor first.
 */
ElevatorAssignment ElevatorAlgorithmD_Assign(const ElevatorState elevators[ELEVATOR_CAR_COUNT],
                                             const ElevatorRequest *request,
                                             const ElevatorTiming *timing);

/* Assign several requests in order and add accepted requests to each car's stop list. */
void ElevatorAlgorithmD_AssignBatch(ElevatorState elevators[ELEVATOR_CAR_COUNT],
                                    const ElevatorRequest *requests,
                                    uint8_t request_count,
                                    const ElevatorTiming *timing,
                                    ElevatorAssignment *assignments);

#endif /* ALGORITHM_D_H */
