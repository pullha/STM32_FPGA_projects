#include "algorithm_b.h"

/* Build one assignment result in a consistent format. */
static ElevatorAssignment MakeAssignment(ElevatorCarId car, bool accepted, const char *reason)
{
  ElevatorAssignment assignment;

  assignment.assigned_car = car;
  assignment.accepted = accepted;
  assignment.reason = reason;

  return assignment;
}

/* Algorithm B car A low-zone floors. Floor 3 is shared. */
static bool LowCarServesFloor(uint8_t floor)
{
  return (floor == 1U) || (floor == 2U) || (floor == 3U);
}

/* Algorithm B car B high-zone floors. Floor 3 is shared. */
static bool HighCarServesFloor(uint8_t floor)
{
  return (floor == 3U) || (floor == 4U) || (floor == 5U);
}

/* A car can handle the request only when both call floor and target floor
 * are inside that car's fixed service zone.
 */
static bool CarCanServeTrip(ElevatorCarId car, const ElevatorRequest *request)
{
  if (car == ELEVATOR_CAR_A)
  {
    return LowCarServesFloor(request->from_floor) && LowCarServesFloor(request->target_floor);
  }

  if (car == ELEVATOR_CAR_B)
  {
    return HighCarServesFloor(request->from_floor) && HighCarServesFloor(request->target_floor);
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

/* Assign one request with algorithm B's low/high zone rule. */
ElevatorAssignment ElevatorAlgorithmB_Assign(const ElevatorState elevators[ELEVATOR_CAR_COUNT],
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

  /* Floor 3 is shared, so the call direction decides the car:
   * 3F DOWN belongs to low-zone car A, and 3F UP belongs to high-zone car B.
   */
  if (request->from_floor == 3U)
  {
    if (request->direction == ELEVATOR_DIR_DOWN)
    {
      return car_a_ok ? MakeAssignment(ELEVATOR_CAR_A, true, "third_floor_down_low_car")
                      : MakeAssignment(ELEVATOR_CAR_NONE, false, "third_floor_down_outside_low_zone");
    }

    if (request->direction == ELEVATOR_DIR_UP)
    {
      return car_b_ok ? MakeAssignment(ELEVATOR_CAR_B, true, "third_floor_up_high_car")
                      : MakeAssignment(ELEVATOR_CAR_NONE, false, "third_floor_up_outside_high_zone");
    }
  }

  /* Outside floor 3, both cars should not normally be valid. If it happens,
   * reject the request instead of silently choosing a car.
   */
  if (car_a_ok && car_b_ok)
  {
    return MakeAssignment(ELEVATOR_CAR_NONE, false, "ambiguous_common_floor");
  }

  if (car_a_ok)
  {
    return MakeAssignment(ELEVATOR_CAR_A, true, "low_floor_zone");
  }

  if (car_b_ok)
  {
    return MakeAssignment(ELEVATOR_CAR_B, true, "high_floor_zone");
  }

  return MakeAssignment(ELEVATOR_CAR_NONE, false, "outside_fixed_zone");
}

/* Assign a whole scenario in order. Each accepted request updates the copied
 * elevator state before the next request is evaluated.
 */
void ElevatorAlgorithmB_AssignBatch(ElevatorState elevators[ELEVATOR_CAR_COUNT],
                                    const ElevatorRequest *requests,
                                    uint8_t request_count,
                                    ElevatorAssignment *assignments)
{
  uint8_t index;

  for (index = 0U; index < request_count; index++)
  {
    assignments[index] = ElevatorAlgorithmB_Assign(elevators, &requests[index]);

    if (assignments[index].accepted)
    {
      ReserveRequest(&elevators[assignments[index].assigned_car], &requests[index]);
    }
  }
}
