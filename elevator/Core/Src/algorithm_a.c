#include "algorithm_a.h"

/* Build one assignment result in a consistent format. */
static ElevatorAssignment MakeAssignment(ElevatorCarId car, bool accepted, const char *reason)
{
  ElevatorAssignment assignment;

  assignment.assigned_car = car;
  assignment.accepted = accepted;
  assignment.reason = reason;

  return assignment;
}

/* Algorithm A car A zone: common floor 1 plus odd floors 3 and 5. */
static bool CarAServesFloor(uint8_t floor)
{
  return (floor == 1U) || (floor == 3U) || (floor == 5U);
}

/* Algorithm A car B zone: common floor 1 plus even floors 2 and 4. */
static bool CarBServesFloor(uint8_t floor)
{
  return (floor == 1U) || (floor == 2U) || (floor == 4U);
}

/* A car can handle the request only when both call floor and target floor
 * are inside that car's fixed service zone.
 */
static bool CarCanServeTrip(ElevatorCarId car, const ElevatorRequest *request)
{
  if (car == ELEVATOR_CAR_A)
  {
    return CarAServesFloor(request->from_floor) && CarAServesFloor(request->target_floor);
  }

  if (car == ELEVATOR_CAR_B)
  {
    return CarBServesFloor(request->from_floor) && CarBServesFloor(request->target_floor);
  }

  return false;
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
 * so later requests in the same scenario see the updated car state.
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

/* Assign one request with algorithm A's fixed odd/even rule. */
ElevatorAssignment ElevatorAlgorithmA_Assign(const ElevatorState elevators[ELEVATOR_CAR_COUNT],
                                             const ElevatorRequest *request)
{
  bool car_a_ok;
  bool car_b_ok;

  if (!ElevatorRequestIsValid(request))
  {
    return MakeAssignment(ELEVATOR_CAR_NONE, false, "invalid_request");
  }

  car_a_ok = CarCanServeTrip(ELEVATOR_CAR_A, request);
  car_b_ok = CarCanServeTrip(ELEVATOR_CAR_B, request);

  /* Floor 1 is shared, but this project requires the chosen button/car to be
   * respected. A floor-1 A call must dispatch A, and a floor-1 B call must dispatch B.
   */
  if ((request->from_floor == 1U) &&
      ((request->requested_car == ELEVATOR_CAR_A) || (request->requested_car == ELEVATOR_CAR_B)))
  {
    if (CarCanServeTrip(request->requested_car, request))
    {
      return MakeAssignment(request->requested_car, true, "first_floor_requested_car");
    }

    return MakeAssignment(ELEVATOR_CAR_NONE, false, "first_floor_requested_car_outside_zone");
  }

  /* If both cars can serve the request and no requested car was given, reject
   * it as ambiguous instead of choosing the nearest car.
   */
  if (car_a_ok && car_b_ok)
  {
    return MakeAssignment(ELEVATOR_CAR_NONE, false, "ambiguous_common_floor");
  }

  if (car_a_ok)
  {
    return MakeAssignment(ELEVATOR_CAR_A, true, "odd_floor_zone");
  }

  if (car_b_ok)
  {
    return MakeAssignment(ELEVATOR_CAR_B, true, "even_floor_zone");
  }

  return MakeAssignment(ELEVATOR_CAR_NONE, false, "outside_fixed_zone");
}

/* Assign a whole scenario in order. Each accepted request updates the copied
 * elevator state before the next request is evaluated.
 */
void ElevatorAlgorithmA_AssignBatch(ElevatorState elevators[ELEVATOR_CAR_COUNT],
                                    const ElevatorRequest *requests,
                                    uint8_t request_count,
                                    ElevatorAssignment *assignments)
{
  uint8_t index;

  for (index = 0U; index < request_count; index++)
  {
    assignments[index] = ElevatorAlgorithmA_Assign(elevators, &requests[index]);

    if (assignments[index].accepted)
    {
      ReserveRequest(&elevators[assignments[index].assigned_car], &requests[index]);
    }
  }
}
