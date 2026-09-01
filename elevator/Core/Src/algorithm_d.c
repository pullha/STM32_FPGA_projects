#include "algorithm_d.h"

/* Default timing for algorithm D when the caller does not provide one:
 * 5 seconds to move one floor and 30 seconds to stop/dwell at each reserved stop.
 */
static const ElevatorTiming DefaultTiming =
{
  5U,
  30U
};

/* Build one assignment result in a consistent format. */
static ElevatorAssignment MakeAssignment(ElevatorCarId car, bool accepted, const char *reason)
{
  ElevatorAssignment assignment;

  assignment.assigned_car = car;
  assignment.accepted = accepted;
  assignment.reason = reason;

  return assignment;
}

/* Use caller-provided timing when available, otherwise use the project default. */
static const ElevatorTiming *GetTimingOrDefault(const ElevatorTiming *timing)
{
  return (timing != 0) ? timing : &DefaultTiming;
}

/* Convert floor distance to travel time in seconds. */
static uint16_t TravelSeconds(uint8_t from_floor, uint8_t to_floor, const ElevatorTiming *timing)
{
  return (uint16_t)ElevatorFloorDistance(from_floor, to_floor) * timing->seconds_per_floor;
}

/* Estimate how many seconds this elevator needs to reach the request floor.
 * Existing reserved stops are simulated first. If the request floor lies on a
 * current segment in the same direction, pickup can happen before the next stop.
 */
static uint16_t EstimatePickupEtaSeconds(const ElevatorState *elevator,
                                         const ElevatorRequest *request,
                                         const ElevatorTiming *timing)
{
  uint8_t index;
  uint8_t position = elevator->current_floor;
  uint16_t elapsed_seconds = 0U;

  if (elevator->stop_count == 0U)
  {
    return TravelSeconds(position, request->from_floor, timing);
  }

  for (index = 0U; index < elevator->stop_count; index++)
  {
    uint8_t next_stop = elevator->stops[index];
    ElevatorDirection segment_direction = ElevatorDirectionBetween(position, next_stop);

    if ((segment_direction == request->direction) &&
        ElevatorFloorBetween(request->from_floor, position, next_stop))
    {
      return (uint16_t)(elapsed_seconds + TravelSeconds(position, request->from_floor, timing));
    }

    elapsed_seconds = (uint16_t)(elapsed_seconds + TravelSeconds(position, next_stop, timing));
    elapsed_seconds = (uint16_t)(elapsed_seconds + timing->dwell_seconds);
    position = next_stop;
  }

  return (uint16_t)(elapsed_seconds + TravelSeconds(position, request->from_floor, timing));
}

/* Add one planned stop while avoiding only immediate duplicate stops. */
static void AppendStop(ElevatorState *elevator, uint8_t floor)
{
  if ((elevator->stop_count > 0U) && (elevator->stops[elevator->stop_count - 1U] == floor))
  {
    return;
  }

  if (elevator->stop_count < ELEVATOR_MAX_STOPS)
  {
    elevator->stops[elevator->stop_count] = floor;
    elevator->stop_count++;
  }
}

/* After a request is accepted, reserve the pickup floor and destination floor
 * so later requests in the same scenario see the updated ETA calculation.
 */
static void ReserveRequest(ElevatorState *elevator, const ElevatorRequest *request)
{
  if (elevator->current_floor != request->from_floor)
  {
    AppendStop(elevator, request->from_floor);
  }

  AppendStop(elevator, request->target_floor);

  if (elevator->direction == ELEVATOR_DIR_IDLE)
  {
    if (elevator->stop_count > 0U)
    {
      elevator->direction = ElevatorDirectionBetween(elevator->current_floor, elevator->stops[0]);
    }

    if (elevator->direction == ELEVATOR_DIR_IDLE)
    {
      elevator->direction = request->direction;
    }
  }
}

/* Assign one request by choosing the car with the shorter estimated pickup time. */
ElevatorAssignment ElevatorAlgorithmD_Assign(const ElevatorState elevators[ELEVATOR_CAR_COUNT],
                                             const ElevatorRequest *request,
                                             const ElevatorTiming *timing)
{
  uint16_t eta_a;
  uint16_t eta_b;
  const ElevatorTiming *active_timing = GetTimingOrDefault(timing);

  if (!ElevatorRequestIsValid(request))
  {
    return MakeAssignment(ELEVATOR_CAR_NONE, false, "invalid_request");
  }

  eta_a = EstimatePickupEtaSeconds(&elevators[ELEVATOR_CAR_A], request, active_timing);
  eta_b = EstimatePickupEtaSeconds(&elevators[ELEVATOR_CAR_B], request, active_timing);

  if (eta_a < eta_b)
  {
    return MakeAssignment(ELEVATOR_CAR_A, true, "eta_priority");
  }

  if (eta_b < eta_a)
  {
    return MakeAssignment(ELEVATOR_CAR_B, true, "eta_priority");
  }

  /* Equal ETA values are resolved by the car with fewer already-reserved stops. */
  if (elevators[ELEVATOR_CAR_A].stop_count <= elevators[ELEVATOR_CAR_B].stop_count)
  {
    return MakeAssignment(ELEVATOR_CAR_A, true, "fewer_requests_tie");
  }

  return MakeAssignment(ELEVATOR_CAR_B, true, "fewer_requests_tie");
}

/* Assign a whole scenario in order. Each accepted request updates the copied
 * elevator state before the next request is evaluated.
 */
void ElevatorAlgorithmD_AssignBatch(ElevatorState elevators[ELEVATOR_CAR_COUNT],
                                    const ElevatorRequest *requests,
                                    uint8_t request_count,
                                    const ElevatorTiming *timing,
                                    ElevatorAssignment *assignments)
{
  uint8_t index;

  for (index = 0U; index < request_count; index++)
  {
    assignments[index] = ElevatorAlgorithmD_Assign(elevators, &requests[index], timing);

    if (assignments[index].accepted)
    {
      ReserveRequest(&elevators[assignments[index].assigned_car], &requests[index]);
    }
  }
}
