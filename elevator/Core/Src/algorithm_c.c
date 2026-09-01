#include "algorithm_c.h"

/* Lower score wins.
 * Same direction and passing the request floor is best, idle is next, and
 * moving away or opposite direction is last.
 */
#define C_SCORE_SAME_DIRECTION  0U
#define C_SCORE_IDLE            100U
#define C_SCORE_OTHER           200U

/* Build one assignment result in a consistent format. */
static ElevatorAssignment MakeAssignment(ElevatorCarId car, bool accepted, const char *reason)
{
  ElevatorAssignment assignment;

  assignment.assigned_car = car;
  assignment.accepted = accepted;
  assignment.reason = reason;

  return assignment;
}

/* Return true when the elevator will pass the request floor if it keeps its
 * current movement direction.
 */
static bool WillPassRequestFloor(const ElevatorState *elevator, const ElevatorRequest *request)
{
  if (elevator->direction == ELEVATOR_DIR_UP)
  {
    return elevator->current_floor <= request->from_floor;
  }

  if (elevator->direction == ELEVATOR_DIR_DOWN)
  {
    return elevator->current_floor >= request->from_floor;
  }

  return false;
}

/* Score one elevator for algorithm C.
 * The main priority is direction/path match. Distance and current request count
 * are added as tie breakers.
 */
static uint16_t ScoreElevator(const ElevatorState *elevator, const ElevatorRequest *request)
{
  uint16_t score;
  uint8_t distance = ElevatorFloorDistance(elevator->current_floor, request->from_floor);

  if ((elevator->direction == request->direction) && WillPassRequestFloor(elevator, request))
  {
    score = C_SCORE_SAME_DIRECTION;
  }
  else if (elevator->direction == ELEVATOR_DIR_IDLE)
  {
    score = C_SCORE_IDLE;
  }
  else
  {
    score = C_SCORE_OTHER;
  }

  score += (uint16_t)distance * 10U;
  score += elevator->stop_count;

  return score;
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

/* Assign one request by comparing algorithm C scores for car A and car B. */
ElevatorAssignment ElevatorAlgorithmC_Assign(const ElevatorState elevators[ELEVATOR_CAR_COUNT],
                                             const ElevatorRequest *request)
{
  uint16_t score_a;
  uint16_t score_b;

  if (!ElevatorRequestIsValid(request))
  {
    return MakeAssignment(ELEVATOR_CAR_NONE, false, "invalid_request");
  }

  score_a = ScoreElevator(&elevators[ELEVATOR_CAR_A], request);
  score_b = ScoreElevator(&elevators[ELEVATOR_CAR_B], request);

  if (score_a < score_b)
  {
    return MakeAssignment(ELEVATOR_CAR_A, true, "nearest_direction_priority");
  }

  if (score_b < score_a)
  {
    return MakeAssignment(ELEVATOR_CAR_B, true, "nearest_direction_priority");
  }

  /* Equal scores are resolved by the car with fewer already-reserved stops. */
  if (elevators[ELEVATOR_CAR_A].stop_count <= elevators[ELEVATOR_CAR_B].stop_count)
  {
    return MakeAssignment(ELEVATOR_CAR_A, true, "fewer_requests_tie");
  }

  return MakeAssignment(ELEVATOR_CAR_B, true, "fewer_requests_tie");
}

/* Assign a whole scenario in order. Each accepted request updates the copied
 * elevator state before the next request is evaluated.
 */
void ElevatorAlgorithmC_AssignBatch(ElevatorState elevators[ELEVATOR_CAR_COUNT],
                                    const ElevatorRequest *requests,
                                    uint8_t request_count,
                                    ElevatorAssignment *assignments)
{
  uint8_t index;

  for (index = 0U; index < request_count; index++)
  {
    assignments[index] = ElevatorAlgorithmC_Assign(elevators, &requests[index]);

    if (assignments[index].accepted)
    {
      ReserveRequest(&elevators[assignments[index].assigned_car], &requests[index]);
    }
  }
}
