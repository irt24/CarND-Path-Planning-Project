#ifndef FST_H
#define FST_H

#include <set>

using namespace std;  // Bad practice :(

const int kSafeDistanceMeters = 30;
const double kInf = std::numeric_limits<double>::max();

struct Predictions {
  double ego_s;
  std::vector<double> other_car_s;
};

class Trajectory {
};

enum State {
  LANE_KEEP,
  PREPARE_LANE_CHANGE_LEFT,
  LANE_CHANGE_LEFT,
  PREPARE_LANE_CHANGE_RIGHT,
  LANE_CHANGE_RIGHT,
};

enum Lane {
  LEFT = 0,
  MIDDLE = 1,
  RIGHT = 2, 
};

class FST {
 public:
  FST(Lane initial_lane) : lane(initial_lane) {}

  double cost1(const Predictions& predictions, const State& next_state) const {
    if (lane == Lane::LEFT) {
      // Keep the lane.
      return next_state == State::LANE_KEEP ? 0.0 : kInf; 
    }

    // We are not in the leftmost lane.
    // Figure out whether any of the other cars are too close.
    for (const double other_s : predictions.other_car_s) {
      if ((other_s > predictions.ego_s) &&
          (other_s - predictions.ego_s < kSafeDistanceMeters)) {
        cout << "Too close. Changing lanes to left." << endl;
        return next_state == State::LANE_CHANGE_LEFT ? 0.0 : kInf;
      }
    }
    return next_state == State::LANE_KEEP ? 0.0 : kInf;
  }

  static double cost2() { return 0.0; }
  static double cost3() { return 0.0; }

  // The transition function.
  State NextState(const Predictions& predictions) {
    State min_cost_state;
    double min_cost = kInf; 

    // Find the state with minimum cost.
    for (const State& state : possible_transitions.find(current_state)->second) {
      double cost = cost1(predictions, state); 
      if (cost < min_cost) {
        min_cost = cost;
        min_cost_state = state;
      }
    };

    current_state = min_cost_state;
    if (current_state == State::LANE_CHANGE_LEFT) {
      lane = Lane::LEFT;
    } else if (current_state == State::LANE_CHANGE_RIGHT) {
      lane = Lane::RIGHT;
    }
    
    return min_cost_state;
  }

  Trajectory GenerateTrajectory(State state) {
    // TODO: Implement this method.
    return Trajectory();
  }

  Lane lane;
  State current_state = LANE_KEEP;

  const map<State, set<State>> possible_transitions = {
    {LANE_KEEP,
        {LANE_KEEP,
         // TODO: Remove LANE_CHANGE_LEFT and LANE_CHANGE_RIGHT.
         LANE_CHANGE_LEFT,
         LANE_CHANGE_RIGHT,
         PREPARE_LANE_CHANGE_LEFT,
         PREPARE_LANE_CHANGE_RIGHT}},
    {PREPARE_LANE_CHANGE_LEFT,
        {LANE_KEEP,
         PREPARE_LANE_CHANGE_LEFT,
         LANE_CHANGE_LEFT}},
    {LANE_CHANGE_LEFT,
        {LANE_CHANGE_LEFT,
         LANE_KEEP}},
    {PREPARE_LANE_CHANGE_RIGHT,
        {LANE_KEEP,
         PREPARE_LANE_CHANGE_RIGHT,
         LANE_CHANGE_RIGHT}},
    {LANE_CHANGE_RIGHT,
        {LANE_CHANGE_RIGHT,
         LANE_KEEP}}
  };
};

#endif  // FST_H
